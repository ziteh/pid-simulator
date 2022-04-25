include("pid_simulator.jl")

using Printf
using Plots

function plot(kp, ki, kd, max=1000.0, min=-1000.0; count=300, interation_time=1.0, init_value=nothing, setpoint_generator=default_setpoint_generator, system_model=default_system, bias=0.0)

  (set_value, actual_value, output, i_term) = pid_run(kp, ki, kd, count, interation_time, setpoint_generator=setpoint_generator, system_model=system_model, max=max, min=min, bias=bias, init_value=init_value)

  # Print info
  @printf("kp: %f, ki: %f, Kd: %f, max: %f, min: %f", kp, ki, kd, max, min)

  # Plot out
  t = 1:count

  p1 = Plots.plot(t, actual_value, label="Actual Value")
  Plots.plot!(t, set_value, label="Set Value", linestyle=:dash)

  p2 = Plots.plot(t, output, label="Output")
  Plots.plot!(t, i_term, label="I Term", ls=:dash)

  Plots.display(Plots.plot(p1, p2, layout=(2, 1)))

  println("")
  print("Done!")
end