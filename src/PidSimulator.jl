module PidSimulator

using Printf
using Plots

include("pid_compute.jl")

export plot, demo, help

function help()
  println("PID Simulator")
  println("- plot(kp, ki, kd)")
  println("- demo()")
  println("- help(): Show this message.")
end

# Generate setpoint data
function default_setpoint_generator(t)
  if t > 200
    return 50.0
  elseif t > 100
    return -5.0
  elseif t > 50
    return 100.0
  else
    return 25.0
  end
end

# Describe how the system responds to the PID output. 
default_system(actual_value, feedback) = actual_value + feedback * 0.3 + 1

function pid_run(
  kp, ki, kd, count, interation_time=1.0;
  setpoint_generator=default_setpoint_generator, system_model=default_system, init_value=nothing, max=typemax(Float64), min=typemin(Float64), bias=0.0)

  set_value = map(setpoint_generator, 1:count)
  actual_value = fill(init_value === nothing ? set_value[1] : init_value, count + 2)
  output = fill(0.0, count)
  i_term = fill(0.0, count + 1)
  error_p = fill(0.0, count + 1)

  for i in 1:count

    (pid_out, i_term_perv, error_perv) = pid_compute(
      set_value[i],
      actual_value[i+1],
      kp, ki, kd,
      i_term[i],
      error_p[i],
      interation_time,
      max=max,
      min=min,
      bias=bias)

    global output[i] = pid_out
    global i_term[i+1] = i_term_perv
    global error_p[i+1] = error_perv

    # PID control is closed-loop control,
    # so the current output(pid_out) will be used as the parameter
    # for the next input(input[i+2]) 
    global actual_value[i+2] = system_model(actual_value[i+1], pid_out)
  end

  return (set_value, actual_value[3:count+2], output, i_term[2:count+1])
end

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
end

function demo()
  # Parameters of PID
  kp_1 = 0.2
  ki_1 = 0.9
  kd_1 = 0.8
  max_1 = 55
  min_1 = -max_1

  kp_2 = 2.0
  ki_2 = 0.8
  kd_2 = 0.8
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

  # Plot out
  t = 1:count

  p1 = Plots.plot(t, actual_value_1, label="Actual Value-1")
  Plots.plot!(t, actual_value_2, label="Actual Value-2")
  Plots.plot!(t, set_value_1, label="Set Value", linestyle=:dash)

  p2 = Plots.plot(t, output_1, label="Output-1", ls=:dash)
  Plots.plot!(t, i_term_1, label="I Term-1")
  Plots.plot!(t, output_2, label="Output-2", ls=:dash)
  Plots.plot!(t, i_term_2, label="I Term-2")

  Plots.display(Plots.plot(p1, p2, layout=(2, 1)))
end

end # module
