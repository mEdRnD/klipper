# Code for handling 8-motor AWD hybrid-corexy IDEX kinematics (Mainline Klipper)
#
# Key architectural modifications:
# 1. Reads 'inverted' as a required parameter from [custom_hybrid_idex] config section.
# 2. rails[0] (stepper_x) loaded via LookupMultiRail to support stepper_x1 (AWD push-pull).
# 3. rails[3] (dual_carriage) loaded via LookupMultiRail to support dual_carriage1 (AWD push-pull).
# 4. rails[1] (stepper_y) loaded via LookupMultiRail to support stepper_y1, stepper_y2, stepper_y3.
# 5. Every stepper on rails[0] and rails[3] is registered with the Y endstop for Markforged homing.
# 6. Signs with inverted: True:
#    - T0 (stepper_x, stepper_x1): corexy_stepper_alloc b'+' (S = X + Y)
#    - T1 (dual_carriage, dual_carriage1): corexy_stepper_alloc b'-' (S = X - Y)
# 7. Inactive Carriage Motion Tracking:
#    DualCarriagesRail wraps each stepper with dual_carriage_alloc. When a carriage is INACTIVE,
#    scale=0.0 and offset=parked_x are applied to its X axis while Y scale remains 1.0.
#    Because AF_Y remains set, Klipper includes the inactive carriage's steppers in the step
#    generation queue for every Y move, commanding them to step by S = parked_x +/- Y, keeping
#    the parked toolhead stationary along X relative to the rail during gantry Y travel.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import traceback
import stepper
from .idex_modes import DualCarriages, DualCarriagesRail

class CustomHybridIDEXKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()

        # Read [custom_hybrid_idex] section as required; missing section raises error
        c_config = config.getsection('custom_hybrid_idex')
        self.inverted = c_config.getboolean('inverted')
        self.printer.add_object('custom_hybrid_idex', self)

        # --- Rail setup via LookupMultiRail ---
        # stepper_x      : LookupMultiRail (stepper_x + stepper_x1)
        # stepper_y      : LookupMultiRail (stepper_y + stepper_y1 + stepper_y2 + stepper_y3)
        # stepper_z      : LookupMultiRail (stepper_z ... stepper_z3 on secondary MCU)
        self.rails = [
            stepper.LookupMultiRail(config.getsection('stepper_x')),
            stepper.LookupMultiRail(config.getsection('stepper_y')),
            stepper.LookupMultiRail(config.getsection('stepper_z')),
        ]

        # Add all T0 carriage steppers to Y endstop
        for s in self.rails[0].get_steppers():
            self.rails[1].get_endstops()[0][0].add_stepper(s)

        # --- Itersolve setup for T0 ---
        # inverted=True  : T0 -> '+' (S = X + Y)
        # inverted=False : T0 -> '-' (S = X - Y)
        t0_sign = b'+' if self.inverted else b'-'
        self.rails[0].setup_itersolve('corexy_stepper_alloc', t0_sign)

        # Cartesian allocation for Y and Z rails
        self.rails[1].setup_itersolve('cartesian_stepper_alloc', b'y')
        self.rails[2].setup_itersolve('cartesian_stepper_alloc', b'z')

        # Axis limits
        ranges = [r.get_range() for r in self.rails]
        self.axes_min = toolhead.Coord([r[0] for r in ranges])
        self.axes_max = toolhead.Coord([r[1] for r in ranges])
        self.dc_module = None

        # --- Dual Carriage setup ---
        if config.has_section('dual_carriage'):
            dc_config = config.getsection('dual_carriage')
            dc_config.getchoice('axis', ['x'], default='x')

            # Load T1 rail using LookupMultiRail to capture dual_carriage + dual_carriage1
            self.rails.append(stepper.LookupMultiRail(dc_config))

            # Add all T1 carriage steppers to Y endstop
            for s in self.rails[3].get_steppers():
                self.rails[1].get_endstops()[0][0].add_stepper(s)

            # --- Itersolve setup for T1 ---
            # inverted=True  : T1 -> '-' (S = X - Y)
            # inverted=False : T1 -> '+' (S = X + Y)
            t1_sign = b'-' if self.inverted else b'+'
            self.rails[3].setup_itersolve('corexy_stepper_alloc', t1_sign)

            # Build DualCarriages module wrapping MultiRail carriages
            dc_rail_0 = DualCarriagesRail(self.rails[0], axis=0, active=True)
            dc_rail_1 = DualCarriagesRail(self.rails[3], axis=0, active=False)
            self.dc_module = DualCarriages(
                dc_config, dc_rail_0, dc_rail_1, axis=0)

        # Register steppers with toolhead motion queue
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())

        self.printer.register_event_handler(
            "stepper_enable:motor_off", self._motor_off)

        # Velocity limits
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', max_velocity, above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat(
            'max_z_accel', max_accel, above=0., maxval=max_accel)

        self.limits = [(1.0, -1.0)] * 3

    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]

    def calc_position(self, stepper_positions):
        pos = [stepper_positions[rail.get_name()] for rail in self.rails]
        dc_status = (self.dc_module.get_status()
                     if self.dc_module is not None else {})
        is_dc_primary = dc_status.get('carriage_1') == 'PRIMARY'

        if is_dc_primary:
            if not self.inverted:
                return [pos[3] - pos[1], pos[1], pos[2]]
            else:
                return [pos[3] + pos[1], pos[1], pos[2]]
        else:
            if not self.inverted:
                return [pos[0] + pos[1], pos[1], pos[2]]
            else:
                return [pos[0] - pos[1], pos[1], pos[2]]

    def update_limits(self, i, range):
        l, h = self.limits[i]
        if l <= h:
            self.limits[i] = range

    def set_position(self, newpos, homing_axes):
        for rail in self.rails:
            rail.set_position(newpos)
        for axis_name in homing_axes:
            if isinstance(axis_name, str):
                axis = 'xyz'.index(axis_name.lower())
            else:
                axis = int(axis_name)
            if self.dc_module and axis == 0:
                rail = self.dc_module.get_primary_rail().get_rail()
            else:
                rail = self.rails[axis]
            if rail is not None:
                self.limits[axis] = rail.get_range()

    def clear_homing_state(self, clear_axes):
        for axis, axis_name in enumerate('xyz'):
            if axis_name in clear_axes:
                self.limits[axis] = (1.0, -1.0)

    def note_z_not_homed(self):
        self.limits[2] = (1.0, -1.0)

    def home_axis(self, homing_state, axis, rail):
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = [None, None, None, None]
        homepos[axis] = hi.position_endstop
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
        else:
            forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
        homing_state.home_rails([rail], forcepos, homepos)

    def home(self, homing_state):
        for axis in homing_state.get_axes():
            if self.dc_module is not None and axis == 0:
                self.dc_module.home(homing_state)
            else:
                self.home_axis(homing_state, axis, self.rails[axis])

    def _motor_off(self, print_time=None):
        self.limits = [(1.0, -1.0)] * 3

    def _check_endstops(self, move):
        end_pos = move.end_pos
        for i in (0, 1, 2):
            if (move.axes_d[i]
                    and (end_pos[i] < self.limits[i][0]
                         or end_pos[i] > self.limits[i][1])):
                if self.limits[i][0] > self.limits[i][1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()

    def check_move(self, move):
        limits = self.limits
        xpos, ypos = move.end_pos[:2]
        if (xpos < limits[0][0] or xpos > limits[0][1]
                or ypos < limits[1][0] or ypos > limits[1][1]):
            self._check_endstops(move)
        if not move.axes_d[2]:
            return
        self._check_endstops(move)
        z_ratio = move.move_d / abs(move.axes_d[2])
        move.limit_speed(
            self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)

    def get_status(self, eventtime):
        axes = [a for a, (l, h) in zip('xyz', self.limits) if l <= h]
        return {
            'homed_axes': ''.join(axes),
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config):
    try:
        return CustomHybridIDEXKinematics(toolhead, config)
    except Exception as e:
        msg = "Custom Hybrid IDEX Kinematics Error: %s\n%s" % (
            str(e), traceback.format_exc())
        logging.error(msg)
        raise config.error(msg)

