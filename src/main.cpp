#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {20, 19},     // Left Chassis Ports (negative port will reverse it!)
    {-13, -12},  // Right Chassis Ports (negative port will reverse it!)

    {14, 15},      // IMU Port
    2.75,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    480);   // Wheel RPM = cartridge * (motor gear / wheel gear)

// Uncomment the trackers you're using here!
// - `8` and `9` are smart ports (making these negative will reverse the sensor)
//  - you should get positive values on the encoders going FORWARD and RIGHT
// - `2.75` is the wheel diameter
// - `4.0` is the distance from the center of the wheel to the center of the robot
ez::tracking_wheel horiz_tracker(18, 2.0, 4.0);  // This tracking wheel is perpendicular to the drive wheels
ez::tracking_wheel vert_tracker(17, 2.0, 0.41);   // This tracking wheel is parallel to the drive wheels

// ez::tracking_wheel left_tracker({'A', 'B'}, 2.75, 4.0);
// ez::tracking_wheel right_tracker({'C', 'D'}, 2.75, 4.0);


//h refers to height target
//r refers to rotation target

double hTarget = 0;
double rTarget = 0.0;
double height = 0;
double angle = 0;
double kG_lift = 0.0;
double kG_arm = 0.0;
double uLift;
double uRotate;
double uFront;
double uBack;
double H_MAX = 2400.0;
double R_MIN = -90.0;
double R_MAX = 60.0;
void liftMove(double lift=0, double turn=0) {
  hTarget = std::clamp(hTarget + lift, 0.0, H_MAX);
  rTarget = std::clamp(rTarget + turn, R_MIN, R_MAX);
  cascadeLift.target_set(hTarget);
  armSpin.target_set(rTarget);
}

void liftTask(void* param) {
  while (true) {    
    // Lift / rope-loop coordinate = average of the two motors in the lift direction.
    // (The old max(-pf, pb) equals lift + |rotation|, so rotating looked like lifting to the PID,
    //  and a motor running ahead in one direction was invisible to it.)
    height = (-liftf.get_position() + liftb.get_position()) / 2.0;
    angle = arm.get_angle() / 100.0;
    angle = (angle > 180.0) ? angle - 360.0 : angle;
    uLift = std::clamp(cascadeLift.compute(height) + kG_lift, -127.0, 127.0);
    uRotate = armSpin.compute(angle) + kG_arm * cos(angle * M_PI / 180.0);

    // The lift channel holds the rope tension, so it gets priority: rotation only gets the power
    // headroom left on each motor. If one motor lags, the lift PID pushes it harder and eases the
    // free motor off, instead of both being scaled down together.
    double headroom = 127.0 - std::abs(uLift);
    uRotate = std::clamp(uRotate, -headroom, headroom);
    uFront = - uLift + uRotate;
    uBack = uLift + uRotate;

    liftf.move(uFront);
    liftb.move(uBack);

    pros::delay(ez::util::DELAY_TIME);
  }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();
  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Look at your horizontal tracking wheel and decide if it's in front of the midline of your robot or behind it
  //  - change `back` to `front` if the tracking wheel is in front of the midline
  //  - ignore this if you aren't using a horizontal tracker
  // chassis.odom_tracker_back_set(&horiz_tracker);
  // Look at your vertical tracking wheel and decide if it's to the left or right of the center of the robot
  //  - change `left` to `right` if the tracking wheel is to the right of the centerline
  //  - ignore this if you aren't using a vertical tracker
  // chassis.odom_tracker_left_set(&vert_tracker);

  // chassis.odom_tracker_left_set(&left_tracker);
  // chassis.odom_tracker_right_set(&right_tracker);

  // Configure your chassis controls
  // chassis.opcontrol_curve_buttons_toggle(true);  // Uncomment to enable modifying the controller curve with buttons on the joysticks (off by default)
  chassis.opcontrol_drive_activebrake_set(0.0);   // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0.0, 0.0);  // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an
                                                  // SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      {"Drive\n\nDrive forward and come back", drive_example},
      {"Turn\n\nTurn 3 times.", turn_example},
      {"Drive and Turn\n\nDrive forward, turn, come back", drive_and_turn},
      {"Drive and Turn\n\nSlow down during drive", wait_until_change_speed},
      {"Swing Turn\n\nSwing in an 'S' curve", swing_example},
      {"Motion Chaining\n\nDrive forward, turn, and come back, but blend everything together :D", motion_chaining},
      {"Combine all 3 movements", combining_movements},
      {"Interference\n\nAfter driving forward, robot performs differently if interfered or not", interfered_example},
      {"Simple Odom\n\nThis is the same as the drive example, but it uses odom instead!", odom_drive_example},
      {"Pure Pursuit\n\nGo to (0, 30) and pass through (6, 10) on the way.  Come back to (0, 0)", odom_pure_pursuit_example},
      {"Pure Pursuit Wait Until\n\nGo to (24, 24) but start running an intake once the robot passes (12, 24)", odom_pure_pursuit_wait_until_example},
      {"Boomerang\n\nGo to (0, 24, 45) then come back to (0, 0, 0)", odom_boomerang_example},
      {"Boomerang Pure Pursuit\n\nGo to (0, 24, 45) on the way to (24, 24) then come back to (0, 0, 0)", odom_boomerang_injected_pure_pursuit_example},
      {"Measure Offsets\n\nThis will turn the robot a bunch of times and calculate your offsets for your tracking wheels.", measure_offsets},
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  cascadeLift.exit_condition_set(100, 20, 300, 60, 200, 200);
  armSpin.exit_condition_set(100, 2, 300, 8, 200, 200);
  liftf.tare_position();
  liftb.tare_position();
  arm.reset();
  // PID targets default to 0, so without this the arm drives toward 0 deg on boot until the first button press.
  cascadeLift.target_set(hTarget);
  armSpin.target_set(rTarget);
  pros::Task lift(liftTask, (void*)"parameter(s) here", "i like lifting");
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  /*
  Odometry and Pure Pursuit are not magic

  It is possible to get perfectly consistent results without tracking wheels,
  but it is also possible to have extremely inconsistent results without tracking wheels.
  When you don't use tracking wheels, you need to:
   - avoid wheel slip
   - avoid wheelies
   - avoid throwing momentum around (super harsh turns, like in the example below)
  You can do cool curved motions, but you have to give your robot the best chance
  to be consistent
  */

  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}

/**
 * Simplifies printing tracker values to the brain screen
 */
void screen_print_tracker(ez::tracking_wheel* tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + ez::util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + ez::util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}

void ez_motor_temperatures() {  // A basic display for chassis motors.
  if (chassis.left_motors.size() * 2 < 12) {
    for (int i = 0; i < chassis.left_motors.size(); i++) {
      ez::screen_print("Motor " + std::to_string(chassis.left_motors[i].get_port()) + ": " +
                           std::to_string(static_cast<int>(chassis.left_motors[i].get_temperature())) + " *C*\t" + "Motor " +
                           std::to_string(chassis.right_motors[i].get_port()) + ": " +
                           std::to_string(static_cast<int>(chassis.right_motors[i].get_temperature())) + " *C",
                       i);  // i iterates the line number
    }
  }
}
/**                   Temperature key
 *  Level 1 - Temp greater than 55 deg C or 131 deg F - 50% Power
 *  Level 2 - Temp greater than 60 deg C or 140 deg F - 25% Power - Less than 25% power is useless on a comp robot
 *  Level 3 - Temp greater than 65 deg C or 149 deg F - 12.5% Power
 *  Level 4 - Temp greater than 70 deg C or 158 deg F - 0% Power
 */

/**
 * Ez screen task
 * Adding new pages here will let you view them during user control or autonomous
 * and will help you debug problems you're having
 */
void ez_screen_task() {
  while (true) {
    // Only run this when not connected to a competition switch
    if (!pros::competition::is_connected()) {
      // Blank page for odom debugging
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        // If we're on the first blank page...
        if (ez::as::page_blank_is_on(0)) {
          // Display X, Y, and Theta
          ez::screen_print("x: " + ez::util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + ez::util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + ez::util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  // Don't override the top Page line

          // Display all trackers that are being used
          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        } else if (true) // ez::as::page_blank_is_on(1)
          // ez_motor_temperatures();
          ez::screen_print("Lift height target: " + ez::util::to_string_with_precision(hTarget) 
                            + "\nArm angle target: " + ez::util::to_string_with_precision(rTarget)
                            + "\nLift height: " + ez::util::to_string_with_precision(height) 
                            + "\nArm angle: " + ez::util::to_string_with_precision(angle)
                            + "\nLift height command: " + ez::util::to_string_with_precision(uLift)
                            + "\nLift turn command: " + ez::util::to_string_with_precision(uRotate),1);
          

        // . . .
        // Add your own blank pages here!
        // . . .
      }
    }

    // Remove all blank pages when connected to a comp switch
    else {
      if (ez::as::page_blank_amount() > 0) ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);

/**
 * Gives you some extras to run in your opcontrol:
 * - run your autonomous routine in opcontrol by pressing DOWN and B
 *   - to prevent this from accidentally happening at a competition, this
 *     is only enabled when you're not connected to competition control.
 * - gives you a GUI to change your PID values live by pressing X
 */
void ez_template_extras() {
  // Only run this when not connected to a competition switch
  if (!pros::competition::is_connected()) {
    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp

    // Enable / Disable PID Tuner
    //  When enabled:
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    if (master.get_digital_new_press(DIGITAL_X)) chassis.pid_tuner_toggle();

    // Trigger the selected autonomous routine
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    // Allow PID Tuner to iterate
    chassis.pid_tuner_iterate();
  }

  // Disable PID Tuner when connected to a comp switch
  else {
    if (chassis.pid_tuner_enabled()) chassis.pid_tuner_disable();
  }
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
int timer = 0;
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  while (true) {
    // Gives you some extras to make EZ-Template ezier
    // ez_template_extras();

    // chassis.opcontrol_tank();  // Tank control
    chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade
    // chassis.opcontrol_arcade_standard(ez::SINGLE);  // Standard single arcade
    // chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE);   // Flipped single arcade

    // . . .
    // Put more user control code here!
    // . . .
    if (master.get_digital_new_press(DIGITAL_X)) {
      claw.set(!claw.get());
    }
    if (timer % 2) {
      if (master.get_digital_new_press(DIGITAL_L1)) {
        liftMove(250.0);
      } else if (master.get_digital_new_press(DIGITAL_L2)) {
        liftMove(-250.0);
      } else if (master.get_digital_new_press(DIGITAL_LEFT)) {
        liftMove(0,5.0);
      } else if (master.get_digital_new_press(DIGITAL_RIGHT)) {
        liftMove(0,-5.0);
      }
    }
    // if (master.get_digital(DIGITAL_L1)) {
    //   liftf.move(-127);
    //   liftb.move(127);
    // } else if (master.get_digital(DIGITAL_L2)) {
    //   liftf.move(127);
    //   liftb.move(-127);
    // } else {
    //   liftf.move(0);
    //   liftb.move(0);
    // }

    if (master.get_digital(DIGITAL_R1)) {
      intake.move(127);
    } else if (master.get_digital(DIGITAL_R2)) {
      intake.move(-127);
    } else {
      intake.move(0);
    }

    timer++;
    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
