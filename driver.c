/*
  driver.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver for Cypress PSoC 5 (CY8CKIT-059)

  Part of grblHAL

  Copyright (c) 2017-2024 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#include "project.h"
#include "serial.h"
#include "driver.h"
#include "grbl/crc.h"
#include "grbl/state_machine.h"

// prescale step counter to 20Mhz (80 / (STEPPER_DRIVER_PRESCALER + 1))
#define STEPPER_DRIVER_PRESCALER 3
#define INTERRUPT_FREQ 1000u
#define SYSTICK_INTERRUPT_VECTOR_NUMBER 15u

static spindle_id_t spindle_id = -1;
static bool spindlePWM = false, IOInitDone = false;
static spindle_pwm_t spindle_pwm = {0};
static axes_signals_t next_step_out;
static delay_t delay = { .ms = 1, .callback = NULL }; // NOTE: initial ms set to 1 for "resetting" systick timer on startup

// Interrupt handler prototypes
static void stepper_driver_isr (void);
//static void stepper_pulse_isr (void);
static void limit_isr (void);
static void control_isr (void);
static void systick_isr (void);

static void driver_delay_ms (uint32_t ms, void (*callback)(void))
{
    if((delay.ms = ms) > 0) {
        DelayTimer_Start();
        if(!(delay.callback = callback)) {
            while(delay.ms)
                grbl.on_execute_delay(state_get());
        }
    } else if(callback)
        callback();
}

// Non-variable spindle

// Start or stop spindle, called from spindle_run() and protocol_execute_realtime()
static void spindleSetStateFixed (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    rpm = rpm;              // stop compiler complaining
    spindle = spindle;
   
    SpindleOutput_Write(state.value);
}

// Variable spindle

// Set spindle speed. Note: spindle direction must be kept if stopped or restarted
static void spindleSetSpeed (spindle_ptrs_t *spindle, uint_fast16_t pwm_value)
{
    if(pwm_value == spindle->context.pwm->off_value) {
        if(spindle->context.pwm->settings->flags.enable_rpm_controlled)
            SpindleOutput_Write(SpindleOutput_Read() & 0x02);
    } else {
        if(!(SpindleOutput_Read() & 0x01))
            SpindleOutput_Write(SpindleOutput_Read() | 0x01);
        SpindlePWM_WriteCompare(pwm_value);
    }
}

static uint_fast16_t spindleGetPWM (spindle_ptrs_t *spindle, float rpm)
{
    return spindle->context.pwm->compute_value(spindle->context.pwm, rpm, false);
}

// Start or stop spindle, called from spindle_run() and protocol_execute_realtime()
static void spindleSetStateVariable (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    uint32_t new_pwm = spindle_pwm.compute_value(spindle->context.pwm, rpm, false);

    if(state.on)
        SpindleOutput_Write(state.ccw ? 0x02 : 0x00);
        
    if(!spindle->context.pwm->settings->flags.enable_rpm_controlled) {
        if(state.on)
            SpindleOutput_Write(state.value);
        else
            SpindleOutput_Write(SpindleOutput_Read() & 0x02); // Keep direction!
    }

    spindleSetSpeed(spindle, new_pwm);
}

bool spindleConfig (spindle_ptrs_t *spindle)
{
    if(spindle == NULL)
        return false;

    if((spindlePWM = spindle_precompute_pwm_values(spindle, &spindle_pwm, &settings.pwm_spindle, hal.f_step_timer))) {
        SpindlePWM_Start();
        SpindlePWM_WritePeriod(spindle_pwm.period);
        spindle->set_state = spindleSetStateVariable;
    } else
        spindle->set_state = spindleSetStateFixed;

    return true;
}

// end Variable spindle

static spindle_state_t spindleGetState (spindle_ptrs_t *spindle)
{   
    return (spindle_state_t)SpindleOutput_Read();
}

// end spindle code

// Enable/disable steppers, called from st_wake_up() and st_go_idle()
static void stepperEnable (axes_signals_t enable, bool hold)
{
    StepperEnable_Write(enable.x);
}

// Sets up for a step pulse and forces a stepper driver interrupt, called from st_wake_up()
// NOTE: delay and pulse_time are # of microseconds
static void stepperWakeUp () 
{
/*
    if(pulse_delay) {
        pulse_time += pulse_delay;
        TimerMatchSet(TIMER2_BASE, TIMER_A, pulse_time - pulse_delay);
    }
*/
    // Enable stepper drivers.
    hal.stepper.enable((axes_signals_t){AXES_BITMASK}, false);
    StepperTimer_WritePeriod(5000); // dummy
    StepperTimer_Enable();
    Stepper_Interrupt_SetPending();
//    hal.stepper_interrupt_callback();

}


// Sets up stepper driver interrupt timeout, called from stepper_driver_interrupt_handler()
static void stepperCyclesPerTick (uint32_t cycles_per_tick)
{
//        StepperTimer_Stop();
//        StepperTimer_WriteCounter(cycles_per_tick < (1UL << 24) /*< 65536 (4.1ms @ 16MHz)*/ ? cycles_per_tick : 0xFFFFFF /*Just set the slowest speed possible.*/);
        StepperTimer_WritePeriod(cycles_per_tick < (1UL << 24) /*< 65536 (4.1ms @ 16MHz)*/ ? cycles_per_tick : 0xFFFFFF /*Just set the slowest speed possible.*/);
//    Control_Reg_1_Write(1);
//    Control_Reg_1_Write(0);
//        StepperTimer_Enable();
}

// Disables stepper driver interrups, called from st_go_idle()
static void stepperGoIdle (bool clear_signals)
{
    StepperTimer_Stop();
    if(clear_signals)
        StepOutput_Write(0);
}

// Sets stepper direction and pulse pins and starts a step pulse
static void stepperPulseStart (stepper_t *stepper)
{
    if(stepper->dir_changed.bits) {
        stepper->dir_changed.bits = 0;
        DirOutput_Write(stepper->dir_out.bits);
    }

    if(stepper->step_out.bits)
        StepOutput_Write(stepper->step_out.bits);
}

// Delayed pulse version: sets stepper direction and pulse pins and starts a step pulse with an initial delay.
// TODO: unsupported, to be completed
static void stepperPulseStartDelayed (stepper_t *stepper)
{
    if(stepper->dir_changed.bits) {
        stepper->dir_changed.bits = 0;
        DirOutput_Write(stepper->dir_out.bits);
    }
    
    if(stepper->step_out.bits) {
        next_step_out = stepper->step_out; // Store out_bits
       
//TODO: implement timer for initial delay...
    }
}

// Enable limit pins interrupt, called from mc_homing_cycle()
static void limitsEnable (bool on, axes_signals_t homing_cycle)
{
    if(on) {
        HomingSignals_WriteMask(~homing_cycle.mask);
        Homing_Interrupt_Enable();
    } else
        Homing_Interrupt_Disable();
}

// Returns limit state as a bit-wise uint8 variable. Each bit indicates an axis limit, where
// triggered is 1 and not triggered is 0. Invert mask is applied. Axes are defined by their
// number in bit position, i.e. Z_AXIS is (1<<2) or bit 2, and Y_AXIS is (1<<1) or bit 1.
inline static limit_signals_t limitsGetState()
{
    limit_signals_t signals;
    
    memset(&signals, 0, sizeof(limit_signals_t));

    signals.min.mask = HomingSignals_Read();

    return signals;
}

static control_signals_t systemGetState (void)
{
    control_signals_t signals;
    
    signals.value = ControlSignals_Read();
    
#ifndef NO_SAFETY_DOOR_SUPPORT
    signals.safety_door_ajar = Off;
#endif

    return signals;
}

// Called by probe_init() and the mc_probe() routines. Sets up the probe pin invert mask to
// appropriately set the pin logic according to setting for normal-high/normal-low operation
// and the probing cycle modes for toward-workpiece/away-from-workpiece.
static void probeConfigure (bool is_probe_away, bool probing)
{
    probing = probing;
    
    ProbeInvert_Write(is_probe_away);
}

// Returns the probe connected and triggered pin states.
probe_state_t probeGetState (void)
{
    probe_state_t state = {
        .connected = On
    };

    state.triggered = ProbeSignal_Read() != 0;

    return state;
}

// Start/stop coolant (and mist if enabled), called by coolant_run() and protocol_execute_realtime()
static void coolantSetState (coolant_state_t mode)
{
    CoolantOutput_Write(mode.value & 0x03);
}

static coolant_state_t coolantGetState (void)
{
    return (coolant_state_t)CoolantOutput_Read();
}

void eepromPutByte (uint32_t addr, uint8_t new_value)
{
    EEPROM_WriteByte(new_value, addr); 
}

nvs_transfer_result_t eepromWriteBlock (uint32_t destination, uint8_t *source, uint32_t size, bool with_checksum)
{
    uint16_t checksum = calc_checksum(source, size);

    for(; size > 0; size--)
        EEPROM_WriteByte(*(source++), destination++); 
    
    if(size > 0 && with_checksum) {
        EEPROM_WriteByte(checksum & 0xFF, destination);
#if NVS_CRC_BYTES > 1
        EEPROM_WriteByte(checksum >> 8, ++destination);
#endif
    }

    return true;
}

nvs_transfer_result_t eepromReadBlock (uint8_t *destination, uint32_t source, uint32_t size, bool with_checksum)
{
    uint32_t remaining = size;

    for(; remaining > 0; remaining--)
        *(destination++) = EEPROM_ReadByte(source++);

#if NVS_CRC_BYTES == 1
    return !with_checksum || calc_checksum(destination, size) == EEPROM_ReadByte(source);
#else
    return !with_checksum || calc_checksum(destination, size) == (EEPROM_ReadByte(source) | (EEPROM_ReadByte(source + 1) << 1));
#endif
}

// Helper functions for setting/clearing/inverting individual bits atomically (uninterruptable)
static void bitsSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    CyGlobalIntDisable;
    *ptr |= bits;
    CyGlobalIntEnable;
}

static uint_fast16_t bitsClearAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    CyGlobalIntDisable;
    uint_fast16_t prev = *ptr;
    *ptr &= ~bits;
    CyGlobalIntEnable;

    return prev;
}

static uint_fast16_t valueSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t value)
{
    CyGlobalIntDisable;
    uint_fast16_t prev = *ptr;
    *ptr = value;
    CyGlobalIntEnable;

    return prev;
}

static void enable_irq (void)
{
    CyGlobalIntEnable;
}

static void disable_irq (void)
{
    CyGlobalIntDisable;
}

// Callback to inform settings has been changed, called by settings_store_global_setting()
// Used to (re)configure hardware and set up helper variables
void settings_changed (settings_t *settings, settings_changed_flags_t changed)
{
    //TODO: disable interrupts while reconfigure?
    if(IOInitDone) {
    
        if(changed.spindle) {
            spindleConfig(spindle_get_hal(spindle_id, SpindleHAL_Configured));
            if(spindle_id == spindle_get_default())
                spindle_select(spindle_id);
        }
    
        if(hal.driver_cap.step_pulse_delay && settings->steppers.pulse_delay_microseconds > 0.0f) {
      //    TimerIntRegister(TIMER2_BASE, TIMER_A, stepper_pulse_isr_delayed);
      //    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT|TIMER_TIMA_MATCH);
            hal.stepper.pulse_start = &stepperPulseStartDelayed;
        }

        StepPulseClock_SetDivider((uint32_t)(24.0f * settings->steppers.pulse_microseconds));

        DirInvert_Write(settings->steppers.dir_invert.mask);
        StepInvert_Write(settings->steppers.step_invert.mask);
        StepperEnableInvert_Write(settings->steppers.enable_invert.x);
        SpindleInvert_Write(settings->pwm_spindle.invert.mask);
        CoolantInvert_Write(settings->coolant.invert.mask);

        // Homing (limit) inputs
        XHome_Write(settings->limits.disable_pullup.x ? 0 : 1);
        XHome_SetDriveMode(settings->limits.disable_pullup.x ? XHome_DM_RES_DWN : XHome_DM_RES_UP);
        YHome_Write(settings->limits.disable_pullup.y ? 0 : 1);
        YHome_SetDriveMode(settings->limits.disable_pullup.y ? YHome_DM_RES_DWN : YHome_DM_RES_UP);
        ZHome_Write(settings->limits.disable_pullup.z ? 0 : 1);
        ZHome_SetDriveMode(settings->limits.disable_pullup.z ? ZHome_DM_RES_DWN : ZHome_DM_RES_UP);
        HomingSignalsInvert_Write(settings->limits.invert.mask);

        // Control inputs
        Reset_Write(settings->control_disable_pullup.reset ? 0 : 1);
        Reset_SetDriveMode(settings->control_disable_pullup.reset ? Reset_DM_RES_DWN : Reset_DM_RES_UP);
        FeedHold_Write(settings->control_disable_pullup.feed_hold ? 0 : 1);
        FeedHold_SetDriveMode(settings->control_disable_pullup.feed_hold ? FeedHold_DM_RES_DWN : FeedHold_DM_RES_UP);
        CycleStart_Write(settings->control_disable_pullup.cycle_start ? 0 : 1);
        CycleStart_SetDriveMode(settings->control_disable_pullup.cycle_start ? CycleStart_DM_RES_DWN : CycleStart_DM_RES_UP);
        SafetyDoor_Write(settings->control_disable_pullup.safety_door_ajar ? 0 : 1);
        SafetyDoor_SetDriveMode(settings->control_disable_pullup.safety_door_ajar ? SafetyDoor_DM_RES_DWN : SafetyDoor_DM_RES_UP);
        ControlSignalsInvert_Write(settings->control_invert.mask);

        // Probe input
        ProbeInvert_Write(settings->probe.disable_probe_pullup ? 0 : 1);
        Probe_SetDriveMode(settings->probe.disable_probe_pullup ? Probe_DM_RES_DWN : Probe_DM_RES_UP);
        Probe_Write(settings->probe.disable_probe_pullup ? 0 : 1);

    }
}

// Initializes MCU peripherals for Grbl use
static bool driver_setup (settings_t *settings)
{
    StepPulseClock_Start();
    StepperTimer_Init();
    Stepper_Interrupt_SetVector(stepper_driver_isr);
    Stepper_Interrupt_SetPriority(1);
    Stepper_Interrupt_Enable();

    Control_Interrupt_StartEx(control_isr);
    ControlSignals_InterruptEnable();
    
    Homing_Interrupt_SetVector(limit_isr);
    
//    CyIntSetSysVector(SYSTICK_INTERRUPT_VECTOR_NUMBER, systick_isr);
//    SysTick_Config(BCLK__BUS_CLK__HZ / INTERRUPT_FREQ);

    DelayTimer_Interrupt_SetVector(systick_isr);
    DelayTimer_Interrupt_SetPriority(7);
    DelayTimer_Interrupt_Enable();
    DelayTimer_Start();

    IOInitDone = settings->version.id == 23;

    hal.settings_changed(settings, (settings_changed_flags_t){0});

    DirOutput_Write(0);

#ifdef HAS_KEYPAD

   /*********************
    *  I2C KeyPad init  *
    *********************/

    I2C_keypad_setup();

#endif

    return IOInitDone;
}

// Initialize HAL pointers
// NOTE: Grbl is not yet (configured from EEPROM data), driver_setup() will be called when done
bool driver_init (void)
{
    EEPROM_Start();

    hal.info = "PSoC 5";
    hal.driver_version = "250327";
    hal.driver_setup = driver_setup;
    hal.f_step_timer = 24000000UL;
    hal.rx_buffer_size = RX_BUFFER_SIZE;
    hal.delay_ms = driver_delay_ms;
    hal.settings_changed = settings_changed;

    hal.stepper.wake_up = stepperWakeUp;
    hal.stepper.go_idle = stepperGoIdle;
    hal.stepper.enable = stepperEnable;
    hal.stepper.cycles_per_tick = stepperCyclesPerTick;
    hal.stepper.pulse_start = stepperPulseStart;

    hal.limits.enable = limitsEnable;
    hal.limits.get_state = limitsGetState;

    hal.coolant.set_state = coolantSetState;
    hal.coolant.get_state = coolantGetState;

    hal.probe.get_state = probeGetState;
    hal.probe.configure = probeConfigure;

    static const spindle_ptrs_t spindle = {
		.type = SpindleType_PWM,
		.ref_id = SPINDLE_PWM0,
		.cap = {
			.direction = On,
			.variable = On,
			.laser = On,
			.gpio_controlled = On
		},
        .config = spindleConfig,
        .set_state = spindleSetStateVariable,
        .get_state = spindleGetState,
        .get_pwm = spindleGetPWM,
        .update_pwm = spindleSetSpeed
    };
    spindle_id = spindle_register(&spindle, "PWM");

    hal.control.get_state = systemGetState;

    memcpy(&hal.stream, serialInit(), sizeof(io_stream_t));

    hal.nvs.type = NVS_EEPROM;
    hal.nvs.get_byte = (uint8_t (*)(uint32_t))&EEPROM_ReadByte;
    hal.nvs.put_byte = eepromPutByte;
    hal.nvs.memcpy_to_nvs = eepromWriteBlock;
    hal.nvs.memcpy_from_nvs = eepromReadBlock;

    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;
    hal.irq_enable = enable_irq;
    hal.irq_disable = disable_irq;

  // driver capabilities, used for announcing and negotiating (with grblHAL) driver functionality

#ifndef NO_SAFETY_DOOR_SUPPORT
    hal.signals_cap.safety_door_ajar = On;
#endif
    hal.limits_cap = (limit_signals_t){ .min.mask = AXES_BITMASK };
    hal.driver_cap.pwm_spindle = On;
    hal.coolant_cap.flood = On;
    hal.coolant_cap.mist = On;
    hal.driver_cap.software_debounce = On;
    hal.driver_cap.step_pulse_delay = Off;
    hal.driver_cap.amass_level = 3;
    hal.driver_cap.control_pull_up = On;
    hal.driver_cap.limits_pull_up = On;
    hal.driver_cap.probe_pull_up = On;

#include "grbl/plugins_init.h"

    // No need to move version check before init.
    // Compiler will fail any signature mismatch for existing entries.
    return hal.version == 10;
}

/* interrupt handlers */

// Main stepper driver
static void stepper_driver_isr (void)
{
    StepperTimer_ReadStatusRegister(); // Clear interrupt

    hal.stepper.interrupt_callback();
}

// This interrupt is enabled when Grbl sets the motor port bits to execute
// a step. This ISR resets the motor port after a short period (settings.pulse_microseconds)
// completing one step cycle.
/*
static void stepper_pulse_isr (void)
{
    //Stepper_Timer_ReadStatusRegister();

    StepOutput_Write(next_step_out.value);
}
*/
static void limit_isr (void)
{
    hal.limits.interrupt_callback(limitsGetState());
}

static void control_isr (void)
{
    control_signals_t signals;
    
    signals.value = ControlSignals_Read();
    
    hal.control.interrupt_callback(signals);
}

// Interrupt handler for 1 ms interval timer
static void systick_isr (void)
{
    DelayTimer_ReadStatusRegister();
    if(!(--delay.ms)) {
        DelayTimer_Stop();
        if(delay.callback) {
            delay.callback();
            delay.callback = NULL;
        }
    }
}
