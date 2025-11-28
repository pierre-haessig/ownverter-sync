/*
 * Copyright (c) 2025 Pierre Haessig
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 2.1 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @brief  Application for islanded inverter (open loop),
 *         with grid synchronization (PLL)
 *         using the three-phase OwnVerter board.
 *
 * @author Pierre Haessig <pierre.haessig@centralesupelec.fr>
 */

/* --------------OWNTECH APIs---------------------------------- */
#include "TaskAPI.h"
#include "ShieldAPI.h"
#include "SpinAPI.h"

/* OWNTECH CONTROL LIBRARY (including trigonometric functions) */
#include "control_factory.h"
#include "transform.h"

/* Oscilloscope library, to record over time the value of selected variables */
#include "Scope.h"

#include "zephyr/console/console.h"

/* -------------- MAIN FUNCTIONS DECLARATION ------------------- */

/* Set up of hardware and software at board startup */
void setup_routine();

/* Interaction with the user on the serial monitor (slow background task) */
void user_interface_task();
/* Displaying board status messages on the serial monitor (slow background task) */
void status_display_task();

/* Power converter control (critical periodic task) */
void control_task();

/* -------------- VARIABLES DECLARATIONS------------------- */

/* --- Control task core variables --- */
static const float32_t T_control = 100e-6F; // Control task period (s) (100µs = 0.1ms)
static const uint32_t T_control_micro = (uint32_t)(T_control * 1.e6F); // Control task period (integer number of µs)
static uint32_t control_count = 0; // disctrete time (=counter) of control task

/* --- Power & safety monitoring constants  --- */
const float32_t GRID_VPK = 21.5F; // Assumed grid voltage amplitude (V) (15.2 Vrms)
const float32_t AC_CURRENT_LIMIT = 3.0; // Max AC current (A)
const float32_t DC_CURRENT_LIMIT = 3.0; // Max DC current (A)
const uint16_t OVER_CURRENT_COUNTER_LIM = 3 ; // Number of successive over current instants before error
const float32_t V_DC = 60; // Assumed DC voltage (V)
const float32_t V_DC_MIN = 2*GRID_VPK; // Minimal DC voltage to start the POWER

/* --- Measurement and Control variables --- */

uint8_t received_serial_char; // Temporary storage for serial monitor character reception
static float32_t meas_data; // Temporary storage for measured value

// DC side variables
static float32_t V_dc; // DC bus voltage (V)
static float32_t I_dc; // Current drawn from the DC bus (A)

// AC side variables
static three_phase_t Vg_abc; // three-phase measured grid voltages (V)
static clarke_t Vg_clarke; // alpha-beta grid voltages (V)
static dqo_t Vg_dq; // dq grid voltages (V)

static three_phase_t Iabc; // three-phase measured injected current (A)
static dqo_t Idq; // dq injected current (A)

static three_phase_t Vi_abc; // three-phase desired inverter voltage (V)
static dqo_t Vi_dq; // dq desired inverter voltage (V)
static three_phase_t duty_abc;

// Vdc lowpass filter
static LowPassFirstOrderFilter vdc_filter = controlLibFactory.lowpassfilter(T_control, 5.0e-3F);
static float32_t V_dc_filt; // DC bus voltage, lowpass filtered (V)
static float32_t inv_V_dc_filt; // 1/Vdc, with a 1/V_DC_MIN bound


float32_t Vi_ref = 0.0;// Inverter d-voltage reference (V)
const float32_t VI_STEP = 1.0; // Inverter voltage step (V)

/* --- Power control & safety monitoring variables  --- */

/* Operation mode control (state machine) */

// Possible operation mode change requests
enum RequestMode {
	IDLE_REQ, // go to IDLE_ST
	POWER_REQ // go to POWER_ST, if possible
};

// Possible operation modes
enum ControlMode {
	IDLE_ST, // Idle mode (no power)
	POWER_ST, // Power mode (PWM active)
	ERROR_ST // Error mode (no power)
};

// Main variables for operation mode control
RequestMode request_mode = IDLE_REQ; // last mode change request
ControlMode control_mode = IDLE_ST; // present converter operation mode
static bool power_on = false; // Power conversion status (PWM activation)

// Power safety monitoring variables
static bool power_ok; // true if POWER mode is fine to run
static uint16_t over_current_counter; // overcurrent error counter
static bool over_current_flag; // memory flag for overcurrent
static bool V_dc_low_flag; // memory flag for low V_high event

/* --- Grid synchronization (PLL) variables --- */
static bool pll_on = false; // ENABLE/DISABLE grid frequency tracking. If false, w=w0

// PLL state (frequency & phase)
const  float32_t GRID_FREQ0 = 50.0; // nominal grid frequency (Hz)
const  float32_t GRID_W0 = 2*PI*GRID_FREQ0; // nominal grid frequency (rad/s)

static float32_t grid_freq = GRID_FREQ0; // grid frequency (Hz)
static float32_t grid_w = GRID_W0; // grid frequency (rad/s)
static float32_t grid_angle = 0.0; // grid angle (rad)
const float32_t PLL_DELTAW_BOUND = 0.2 * GRID_W0; // +/- bound on PLL frequency deviation (rad/s) (0.2 F0 = 10 Hz)

// PLL monitoring
static bool pll_synced; // PLL sync status
static bool pll_phase_ok; // phase error low enough
static bool pll_freq_ok; // frequency deviation around nominal low enough
static bool pll_ok; // all PLL deviations low enoug
static bool pll_unsync_flag; // memory flag for PLL unsync event
static uint32_t pll_sync_counter; // PLL sync and unsync delay counter
const uint32_t  PLL_SYNC_COUNTER_TARGET = 10e-3/T_control; // 10 ms sync acceptance delay
const uint32_t  PLL_UNSYNC_COUNTER_TARGET = 1e-3/T_control; // 1 ms unsync delay
const float32_t PLL_VQ_SYNC_TOLERANCE = 0.5*GRID_VPK; // Vgrid_Q  voltage tolerance (large due to clipped voltages) (V)
const float32_t PLL_DELTAW_SYNC_TOLERANCE = 0.5*PLL_DELTAW_BOUND;// deviation tolerance from w0, rad/s

// PLL PI feedback filter and parameters
const float32_t RISE_TIME_PLL = 3.0F / GRID_FREQ0; // desired PLL rise time (s): 3 periods
const float32_t WN_PLL = 3.0F / RISE_TIME_PLL; // desired PLL 2nd order natural frequency (rad/s)
const float32_t XSI_PLL = 0.7F; // desired damping factor
const float32_t KP_PLL = 2 * WN_PLL * XSI_PLL / GRID_VPK; // Proportional feedback gain
const float32_t KI_PLL = (WN_PLL * WN_PLL) / GRID_VPK; // Integral feedback gain
const float32_t TI_PLL = KP_PLL / KI_PLL; // Integral feedback time constant (s)
static Pid pi_pll; // PI feedback filter of the PLL

/* --- Scope, to record over time the value of selected variables --- */

// Scope variables and parameters
const uint16_t SCOPE_LENGTH = 200; // number of instants to record
const uint32_t SCOPE_CHANNELS = 15; // number of variables to record (needs to be coherent with setup_scope!)
const uint32_t SCOPE_DECIMATION = 2; // acquisition rate decimation, with respect to control task period
const float32_t SCOPE_PRETRIG = 5.0/40.0; // record 12.5% samples before trigger (5 ms)

Scope scope(SCOPE_LENGTH, SCOPE_CHANNELS, T_control*SCOPE_DECIMATION);
static bool dump_scope_req; // request dump of scope data

// Scope functions: trigger setup, acquire and print

/* Scope triggering function: return true to start acquisition */
bool scope_trigger() {
	return pll_on;
}

/* Scope channels and trigger configuration

with number of channels:
- all channels: 1+7+10+3+2 = 23
- no inverter voltages and duties: 1+7+5+2 = 15
*/
void setup_scope() {
	// DC bus voltage (1 ch)
	scope.connectChannel(V_dc, "Vdc");

	// Grid voltages (7 ch)
	scope.connectChannel(Vg_abc.a, "Vgrid_a");
	scope.connectChannel(Vg_abc.b, "Vgrid_b");
	scope.connectChannel(Vg_abc.c, "Vgrid_c");

	scope.connectChannel(Vg_clarke.alpha, "Vgrid_alpha");
	scope.connectChannel(Vg_clarke.beta, "Vgrid_beta");
	scope.connectChannel(Vg_dq.d, "Vgrid_d");
	scope.connectChannel(Vg_dq.q, "Vgrid_q");

	// Inverter currents (5 ch)
	scope.connectChannel(Iabc.a, "Ia");
	scope.connectChannel(Iabc.b, "Ib");
	scope.connectChannel(Iabc.c, "Ic");
	scope.connectChannel(Idq.d, "Id");
	scope.connectChannel(Idq.q, "Iq");

	// Inverter voltages (5 ch)
	// scope.connectChannel(Vi_abc.a, "Vinv_a");
	// scope.connectChannel(Vi_abc.b, "Vinv_b");
	// scope.connectChannel(Vi_abc.c, "Vinv_c");
	// scope.connectChannel(Vi_dq.d, "Vinv_d");
	// scope.connectChannel(Vi_dq.q, "Vinv_q");

	// // Duty cycles (3 ch)
	// scope.connectChannel(duty_abc.a, "duty_a");
	// scope.connectChannel(duty_abc.b, "duty_b");
	// scope.connectChannel(duty_abc.c, "duty_c");

	// PLL variables (2 ch)
	scope.connectChannel(grid_freq, "freq");
	scope.connectChannel(grid_angle, "angle");

	scope.set_trigger(&scope_trigger);
	scope.set_pretrig_ratio(SCOPE_PRETRIG);
	scope.start();
}

void print_scope_record() {
	printk("begin record\n");
	scope.init_dump(); // Init the data dump process
	while (scope.get_dump_state() != DUMP_FINISHED) {
		printk("%s", scope.dump());
		task.suspendBackgroundUs(200);
	}
	printk("end record\n");
}

/* -------------- PLL FUNCTIONS ------------------------------- */

/*Init (reset) PLL state variables and PI controller */
void setup_PLL() {
	/*State variables*/
	grid_angle = 0.0;
	pll_on = false;
	pll_synced = false;
	pll_unsync_flag = false;
	pll_sync_counter = 0;

	// PLL PI feedback filter
	pi_pll = controlLibFactory.pid(T_control, KP_PLL, TI_PLL, 0.0, 0.0, -PLL_DELTAW_BOUND, PLL_DELTAW_BOUND);

	pi_pll.reset(0.0); // init frequency w=w0
}

/* Update PLL state and frequency */
inline void run_grid_PLL() {
	if (pll_on) {
		// PLL frequency update
		float32_t grid_deltaw = pi_pll.calculateWithReturn(0, -Vg_dq.q); // Delta w. use -Vq as measurement so that process error = +Vq
		grid_w = GRID_W0 + grid_deltaw;
		// PLL status monitor
		pll_phase_ok = Vg_dq.q < PLL_VQ_SYNC_TOLERANCE && Vg_dq.q > -PLL_VQ_SYNC_TOLERANCE; // instantaneous PLL phase error below tolerance
		pll_freq_ok = grid_deltaw < PLL_DELTAW_SYNC_TOLERANCE && grid_deltaw > -PLL_DELTAW_SYNC_TOLERANCE;
		pll_ok = pll_phase_ok && pll_freq_ok;

		if (!pll_synced) { // UNSYNCED PLL state
			if (pll_ok) {
				pll_sync_counter += 1; // accumulate successive phase_ok
			}
			else { // phase || freq not ok
				pll_sync_counter = 0; // reset counter
			}
			// PLL sync state update
			if (pll_sync_counter >= PLL_SYNC_COUNTER_TARGET) {
				pll_synced = true;
				pll_sync_counter = 0; // reset counter for SYNCED state
			}
		} else { // SYNCED PLL state
			if (!pll_ok) {
				pll_sync_counter += 1; // accumulate successive !pll_ok
			}
			else { // phase && freq ok
				pll_sync_counter = 0; // reset counter
			}
			// PLL sync state update
			if (pll_sync_counter >= PLL_UNSYNC_COUNTER_TARGET) {
				pll_synced = false;
				pll_unsync_flag = true; // raise PLL unsync event flag
				pll_sync_counter = 0; // reset counter
			}
		}
	} // PLL OFF: grid_w stays constant at w0
	else {
		grid_w = GRID_W0;
		pll_sync_counter = 0;
		pll_synced = false;
		pi_pll.reset(0.0);
	}
	// update angle (always, no matter PLL ON/OFF status)
	grid_angle = ot_modulo_2pi(grid_angle + grid_w*T_control);
	// Hz frequency output
	grid_freq = grid_w/(2*PI);
}

/* -------------- SETUP FUNCTION -------------------------------*/

/**
 * Main setup routine, called at board startup.
 * It is used to initialize the board (spin microcontroller and power shield)
 * and the application (set tasks).
 */
void setup_routine()
{
	spin.led.turnOn(); // LED ON at board startup

	/* Set the high switch convention for all legs */
	shield.power.initBuck(ALL);
	shield.power.setDutyCycleMin(ALL, 0.0);
	shield.power.setDutyCycleMax(ALL, 1.0);

	/* Setup all the measurements */
	shield.sensors.enableDefaultOwnverterSensors();

	setup_scope();
	setup_PLL();

	/* Declare tasks */
	uint32_t app_task_number = task.createBackground(status_display_task);
	uint32_t com_task_number = task.createBackground(user_interface_task);
	task.createCritical(control_task, T_control_micro);

	/* Start tasks */
	task.startBackground(app_task_number);
	task.startBackground(com_task_number);
	task.startCritical();
}

/* -------------- BACKGROUND TASKS ------------------------------- */

/* Clear power and PLL error flags */
void clear_error_flags();

/**
 * User interface task, running in a loop in the background.
 * It allows controlling the application through the serial monitor.
 *
 * It waits for the user to press a key to select an action.
 * In particular, 'h' displays the help menu.
 */
void user_interface_task()
{
	// Task content
	received_serial_char = console_getchar();
	switch (received_serial_char) {
	case 'h':
		printk("Command help\n");
		printk("- p/i: Request Power/Idle mode change\n");
		printk("- a:   Clear (Acknowledge) power error flags\n");
		printk("- l:   Toggle PLL ON/OFF\n");
		printk("- u/j: Increase/Decrease voltage reference by (%.1f) V\n", (double)VI_STEP);
		printk("- s:   Retreive latest scope data\n");
		printk("- r:   Relaunch scope acquisition\n");
		break;
	// Operation mode
	case 'p':
		printk("POWER mode requested\n");
		request_mode = POWER_REQ;
		break;
	case 'i':
		printk("IDLE mode requested\n");
		request_mode = IDLE_REQ;
		break;
	case 'a':
		printk("Clearing power error flags\n");
		clear_error_flags();
		break;
	// PLL toggle
	case 'l':
		if (pll_on) {pll_on = false;}
		else {pll_on = true;}
		printk("PLL toggled %s\n", pll_on ? "ON" : "OFF");
		break;
	// Setpoint changes
	case 'u':
		printk("Voltage reference UP\n");
		Vi_ref += VI_STEP;
        break;
    case 'j':
		printk("Voltage reference DOWN\n");
        Vi_ref -= VI_STEP;
        break;
	// Scope
	case 's':
		printk("Retreive latest scope data\n");
		dump_scope_req = true;
		break;
	case 'r':
		printk("Relaunch scope acquisition\n");
		scope.start();
		break;
	}
}

/**
 * Board status display task, called pseudo-periodically.
 * It displays board measurements on the serial monitor
 *
 * It also sets the board LED (blinking when POWER_MODE)
 * and print (dump) scope data when requested
 */
void status_display_task()
{
	// Operation mode display
	switch (control_mode) {
		case IDLE_ST:
			spin.led.turnOn(); // Constantly ON led when IDLE
			printk("IDL: ");
			break;

		case POWER_ST:
			spin.led.toggle(); // Blinking LED when POWER
			printk("POW: ");
			break;

		case ERROR_ST:
			spin.led.turnOn(); // Short flash LED when ERROR
			printk("ERR: ");
			spin.led.turnOff();
			break;
	}

	// PLL display:
	printk("PLL %s (%s %.1f Hz) | ", pll_on ? "ON ":"OFF", pll_synced ? "SYNCED":"UNSYNC", (double) grid_freq);
	// Display various measurements
	printk("Vref %4.1f V, ", (double) Vi_ref);
	printk("Vgd %4.1f V, ", (double) Vg_dq.d);
	printk("Idq %5.1f,%5.1f A, ", (double)Idq.d, (double)Idq.d);
	printk("Vdc %4.1f V, ", (double) V_dc);
	printk("Idc %4.1f A | ", (double) I_dc);
	// Scope
	printk("Scope %s | ", scope.acq_state() == ACQ_DONE ? "DONE" : "....");
	// Error flags display:
	if (over_current_flag || V_dc_low_flag || pll_unsync_flag) {
		printk("Err: ");
		if (over_current_flag) {
			printk("OC ");
		}
		if (V_dc_low_flag) {
			printk("VDC ");
		}
		if (pll_unsync_flag) {
			printk("PLL");
		}
	}
	printk("\n");

	/* Print scope data, if requested */
	if (dump_scope_req) {
		if (scope.acq_state() == ACQ_DONE) {
			print_scope_record();
		}
		else {
			printk("Scope acquisition not DONE, no data to dump\n");
		}
		dump_scope_req = false;
	}

	task.suspendBackgroundMs(200);
}

/* -------------- CRITICAL CONTROL TASK & SUBFUNCTIONS ----------------------- */

/* Read measurements from analog sensors, possibly applying some filters,
   through microcontroller ADCs (Analog to Digital Converters).

   Measured signals:
   - currents: Iabc, I_dc
   - voltages: Vg_abc, V_dc (with V_dc lowpass filtered version)
 */
inline void read_measurements()
{
	// DC side measurements
	meas_data = shield.sensors.getLatestValue(I_HIGH);
	if (meas_data != NO_VALUE) {
		I_dc = meas_data;
	}

	meas_data = shield.sensors.getLatestValue(V_HIGH);
	if (meas_data != NO_VALUE) {
		V_dc = meas_data;
	}

	// AC side measurements
	meas_data = shield.sensors.getLatestValue(V1_LOW);
	if (meas_data != NO_VALUE) {
		Vg_abc.a = meas_data;
	}

	meas_data = shield.sensors.getLatestValue(V2_LOW);
	if (meas_data != NO_VALUE) {
		Vg_abc.b = meas_data;
	}

	meas_data = shield.sensors.getLatestValue(V3_LOW);
	if (meas_data != NO_VALUE) {
		Vg_abc.c = meas_data;
	}
	// Currents
	meas_data = shield.sensors.getLatestValue(I1_LOW);
	if (meas_data != NO_VALUE) {
		Iabc.a = meas_data;
	}

	meas_data = shield.sensors.getLatestValue(I2_LOW);
	if (meas_data != NO_VALUE) {
		Iabc.b = meas_data;
	}

	meas_data = shield.sensors.getLatestValue(I3_LOW);
	if (meas_data != NO_VALUE) {
		Iabc.c = meas_data;
	}

	/* Apply filters */
	// Smooth V_dc (lowpass)
	V_dc_filt = vdc_filter.calculateWithReturn(V_dc);
}

/* Transform three-phase measurements to Clarke & DQ */
inline void transform_3ph_measurements(float32_t grid_angle) {
	Vg_clarke  = Transform::clarke(Vg_abc);
	Vg_dq = Transform::rotation_to_dqo(Vg_clarke, grid_angle);
	Idq = Transform::to_dqo(Iabc, grid_angle);
}

/* Monitor currents and return true if too high during successive instants → over_current */
inline bool over_current_monitor() {
	bool over_current_inst = Iabc.a > AC_CURRENT_LIMIT || Iabc.a < -AC_CURRENT_LIMIT ||
	    Iabc.b > AC_CURRENT_LIMIT || Iabc.b < -AC_CURRENT_LIMIT ||
		Iabc.c > AC_CURRENT_LIMIT || Iabc.c < -AC_CURRENT_LIMIT ||
	    I_dc > DC_CURRENT_LIMIT; // instantaneous overcurrent
	if (over_current_inst) {
		over_current_counter++;
	}
	if (over_current_counter >= OVER_CURRENT_COUNTER_LIM) {
		return true;
	} else {
		return false;
	}
}

/* Monitor power related variables to allow POWER mode → power_ok

depends on Vdc voltage, currents and PLL status
*/
inline bool monitor_power() {
	bool over_current = over_current_monitor();
	// Memorize overcurrent error flag
	if (over_current) {over_current_flag = true;}

	bool V_dc_low = V_dc_filt < V_DC_MIN;
	// Memorize Vdc low error flag
	if (V_dc_low) {V_dc_low_flag = true;}

	// return power_ok signal, including PLL status
	//return  !V_dc_low && pll_synced && !over_current;
	return  !V_dc_low && !over_current; // PLL sync requirement removed from power_ok
}

/* Clear power and PLL error flags */
void clear_error_flags() {
	over_current_flag = false;
	V_dc_low_flag = false;
	pll_unsync_flag = false;
}

/* Compute inverter voltage.

In this open loop version, it is juste equal Vd set point.
In close loop current control version, it will depend on the current set point and measurement */
void compute_inverter_voltages(float32_t grid_angle) {
	Vi_dq.d = Vi_ref;
	Vi_dq.q = 0.0F;
	// Transform back to abc
	Vi_abc = Transform::to_threephase(Vi_dq, grid_angle);
}

/* Convert inverter leg voltage to duty cycle, including saturation

Leg voltage in the [-Vdc/2, +Vdc/2] interval is mapped to [0,1],
meaning that the duty cycle offset is added automatically.
*/
inline float32_t voltage_to_duty(float32_t Vleg, float32_t inverse_Vhigh)
{
	static float32_t duty_raw;
	const float32_t duty_offset = 0.5F;
	duty_raw = Vleg * inverse_Vhigh + duty_offset;
	if (duty_raw > 1.0F) {
		return 1.0F;
	}
	else if (duty_raw < 0.0F) {
		return 0.0F;
	}
	else {
		return duty_raw;
	}
}

/* Compute duty_abc from Vi_abc and V_high_filtered */
inline void compute_duties()
{
	// Invert Vdc with a bound
	if (V_dc_filt > V_DC_MIN ) {
		inv_V_dc_filt = 1.0F / V_dc_filt;
	}
	else {
		inv_V_dc_filt = 1.0F / V_dc_filt;
	}

	duty_abc.a = voltage_to_duty(Vi_abc.a, inv_V_dc_filt);
	duty_abc.b = voltage_to_duty(Vi_abc.b, inv_V_dc_filt);
	duty_abc.c = voltage_to_duty(Vi_abc.c, inv_V_dc_filt);
}

/* Apply legs duty cycles to the PWM generators */
inline void apply_duties()
{
	shield.power.setDutyCycle(LEG1, duty_abc.a);
	shield.power.setDutyCycle(LEG2, duty_abc.b);
	shield.power.setDutyCycle(LEG3, duty_abc.c);
}

/**
 * This is the code loop of the critical task.
 * It is executed every T_control seconds (100 µs by default).
 *
 * Actions:
 * - measure voltage and currents
 * - compute frequency (PLL)
 * - compute inverter voltages & duty cycles
 * - control the power converter leg (ON/OFF state)
 */
void control_task() {

	// Signal processing operations: measurements, monitoring and PLL
	run_grid_PLL(); // Note: chicken and egg problem about which angle to use to Park-transform Vgrid and which Vg.q to use for PLL
	read_measurements();
	transform_3ph_measurements(grid_angle);

	// Operation mode: state change logic
	power_ok =  monitor_power();
	switch (control_mode) {
		case IDLE_ST:
			if ((request_mode == POWER_REQ) && power_ok) {
				control_mode = POWER_ST;
			}
			break;
		case POWER_ST:
			if (!power_ok) {
				control_mode = ERROR_ST;
			} else if (request_mode == IDLE_REQ) {
				control_mode = IDLE_ST;
			}
			break;
		case ERROR_ST:
			if (request_mode == IDLE_REQ) {
				// automatically reset error flags and counters when going to IDLE
				clear_error_flags();
				control_mode = IDLE_ST;
			}
			break;
	}

	// Operation mode: state action logic
	switch (control_mode) {
		case IDLE_ST:
		case ERROR_ST:
			// Stop power if it's ON
			if (power_on) {
				shield.power.stop(ALL);
				power_on = false;
			}
			break;
		case POWER_ST:
			compute_inverter_voltages(grid_angle);
			compute_duties();
			apply_duties();
			// Start power if it's OFF
			if (!power_on) {
				power_on = true;
				shield.power.start(ALL);
			}
			break;
	}

	// Scope variables capture
	if (control_count % SCOPE_DECIMATION == 0) {
		scope.acquire();
	}

	// Increment time for next iteration
	control_count++;
}

/**
 * Main function of the application.
 * This function is generic and does not need editing.
 */
int main(void)
{
	setup_routine();
	return 0;
}