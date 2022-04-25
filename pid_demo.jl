include("pid_simulator.jl")

using Printf
using Plots

# Parameters of PID
kp_1 = 0.4
ki_1 = 0.9
kd_1 = 0.5
max_1 = 55
min_1 = -max_1

kp_2 = 1.5
ki_2 = 0.2
kd_2 = 0.5
max_2 = 55
min_2 = -max_2

count = 300

spg = default_setpoint_generator
sys = default_system

(set_value_1, actual_value_1, output_1, i_term_1) = pid_run(
  kp_1, ki_1, kd_1, count, 1, setpoint_generator=spg, system_model=sys, max=max_1, min=min_1)

(set_value_2, actual_value_2, output_2, i_term_2) = pid_run(
  kp_2, ki_2, kd_2, count, 1, setpoint_generator=spg, system_model=sys, max=max_2, min=min_2)

# Print info
println("--- PID Simulator Demo ---")
println("1st config:")
@printf("kp: %f, ki: %f, Kd: %f, max: %f, min: %f", kp_1, ki_1, kd_1, max_1, min_1)
println("")

println("2nd config:")
@printf("kp: %f, ki: %f, Kd: %f, max: %f, min: %f", kp_2, ki_2, kd_2, max_2, min_2)
println("")

# Plot out
t = 1:count

p1 = Plots.plot(t, set_value_1, label="Set Value", lw=2, ls=:dash)
Plots.plot!(t, actual_value_1, label="Actual Value-1")
Plots.plot!(t, actual_value_2, label="Actual Value-2")

p2 = Plots.plot(t, output_1, label="Output-1", ls=:dash, leg=:bottomright)
Plots.plot!(t, i_term_1, label="I Term-1")
Plots.plot!(t, output_2, label="Output-2", ls=:dash)
Plots.plot!(t, i_term_2, label="I Term-2")

Plots.display(Plots.plot(p1, p2, layout=(2, 1)))

print("Done!")