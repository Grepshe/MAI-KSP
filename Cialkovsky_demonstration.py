import math
import time
import krpc

turn_start_altitude = 250
turn_end_altitude = 45000
target_altitude = 150000

conn = krpc.connect(name='Cialkovsky')
vessel = conn.space_center.active_vessel

ut = conn.add_stream(getattr, conn.space_center, 'ut')
altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
stage_2_resources = vessel.resources_in_decouple_stage(stage=2, cumulative=False)
srb_fuel = conn.add_stream(stage_2_resources.amount, 'SolidFuel')

vessel.control.sas = False
vessel.control.rcs = False
vessel.control.throttle = 1.0

time.sleep(1)
print('Запуск')

vessel.control.activate_next_stage()
vessel.auto_pilot.engage()
vessel.auto_pilot.target_pitch_and_heading(90, 90)

srbs_separated = False
turn_angle = 0
while True:

    if altitude() > turn_start_altitude and altitude() < turn_end_altitude:
        frac = ((altitude() - turn_start_altitude) /
                (turn_end_altitude - turn_start_altitude))
        new_turn_angle = frac * 90
        if abs(new_turn_angle - turn_angle) > 0.5:
            turn_angle = new_turn_angle
            vessel.auto_pilot.target_pitch_and_heading(90-turn_angle, 90)

    if not srbs_separated:
        if srb_fuel() < 0.1:
            vessel.control.activate_next_stage()
            srbs_separated = True
            print('Ускорители отделены')

    if apoapsis() > target_altitude*0.9:
        break

vessel.control.throttle = 0.25
while apoapsis() < target_altitude:
    pass
vessel.control.throttle = 0.0

print('Выход из атмосферы')
while altitude() < 70500:
    pass

print('Расчёт скорости для выхода на орбиту')
mu = vessel.orbit.body.gravitational_parameter
r = vessel.orbit.apoapsis
a1 = vessel.orbit.semi_major_axis
a2 = r
v1 = math.sqrt(mu*((2./r)-(1./a1)))
v2 = math.sqrt(mu*((2./r)-(1./a2)))
delta_v = v2 - v1
node = vessel.control.add_node(
    ut() + vessel.orbit.time_to_apoapsis, prograde=delta_v)

print("Расчитываем время сжигания используя формулу Циалковского (deltaV = Isp*g*ln(m0/mf)):")
F = vessel.available_thrust
print("Доступная тяга (F) =", F)
Isp = vessel.specific_impulse * 9.82
print("Удельный импульс (Isp) =", Isp)
m0 = vessel.mass
print("Полная масса (m0) =", m0)
m1 = m0 / math.exp(delta_v/Isp)
print("Пустая масса mf =", m1)
flow_rate = F / Isp
print("Скорость сжигания: F/Isp = ", flow_rate)
burn_time = (m0 - m1) / flow_rate
print("Таким образом необходимое время сжигания:", burn_time)

print('Разворот для выхода на орбиту')
vessel.auto_pilot.reference_frame = node.reference_frame
vessel.auto_pilot.target_direction = (0, 1, 0)
vessel.auto_pilot.wait()

print('Выхода на орбиту...')
burn_ut = ut() + vessel.orbit.time_to_apoapsis - (burn_time/2.)
lead_time = 5
conn.space_center.warp_to(burn_ut - lead_time)

time_to_apoapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')
while time_to_apoapsis() - (burn_time/2.) > 0:
    pass
vessel.control.throttle = 1.0
time.sleep(burn_time - 0.1)
vessel.control.throttle = 0.05
remaining_burn = conn.add_stream(node.remaining_burn_vector, node.reference_frame)
while remaining_burn()[1] > 0:
    pass
vessel.control.throttle = 0.0
node.remove()

print('Выход на орбиту завершён')
