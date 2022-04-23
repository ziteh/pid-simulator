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
    return 50
  elseif t > 100
    return -5
  elseif t > 50
    return 100
  else
    return 25
  end
end

# Describe how the system responds to the PID output. 
default_system(lastInput, feedback) = lastInput + feedback * 0.3 + 1

function pid_run(kp, ki, kd, init_input, count, setpoint_generator, system_model; max=typemax(Float64), min=typemin(Float64))
  setpoint = map(setpoint_generator, 1:count)
  input = fill(init_input, count + 2)
  output = fill(0.0, count)
  iterm = fill(0.0, count + 1)

  for i in 1:count
    (pid_out, l_iterm, l_in) = pid_compute(
      setpoint[i],
      input[i+1],
      kp, ki, kd,
      iterm[i],
      input[i],
      max,
      min)

    global iterm[i+1] = l_iterm
    global output[i] = pid_out

    # PID control is closed-loop control,
    # so the current output(pid_out) will be used as the parameter
    # for the next input(input[i+2]) 
    global input[i+2] = system_model(l_in, pid_out)
  end

  return (setpoint, input[3:count+2], output, iterm[2:count+1])
end

function plot(kp, ki, kd, max=1000.0, min=-1000.0; count=300, init_input=25.0, setpoint_generator=default_setpoint_generator, system_model=default_system)
  (setpoint, input, output, iterm) = pid_run(kp, ki, kd, init_input, count, setpoint_generator, system_model, max=max, min=min)

  # Print info
  @printf("kp: %f, ki: %f, Kd: %f, max: %f, min: %f", kp, ki, kd, max, min)

  # Plot out
  t = 1:count

  p1 = Plots.plot(t, input, label="Input")
  Plots.plot!(t, setpoint, label="Setpoint", linestyle=:dash)

  p2 = Plots.plot(t, output, label="Output")
  Plots.plot!(t, iterm, label="I Term", ls=:dash)

  Plots.display(Plots.plot(p1, p2, layout=(2, 1)))
end

function demo()
  # Parameters of PID
  kp_1 = 150 / 100
  ki_1 = 20 / 100
  kd_1 = 80 / 100
  max_1 = 25
  min_1 = -max_1

  kp_2 = 250 / 100
  ki_2 = 4 / 100
  kd_2 = 5 / 100
  max_2 = 25
  min_2 = -max_2

  count = 300
  init_input = 25.0

  spg = default_setpoint_generator
  sys = default_system

  (setPoint_1, input_1, output_1, iTerm_1) = pid_run(kp_1, ki_1, kd_1, init_input, count, spg, sys, max=max_1, min=min_1)

  (setPoint_2, input_2, output_2, iTerm_2) = pid_run(kp_2, ki_2, kd_2, init_input, count, spg, sys, max=max_2, min=min_2)

  # Print info
  println("--- PID Simulator Demo ---")
  println("1st config:")
  @printf("kp: %f, ki: %f, Kd: %f, max: %f, min: %f", kp_1, ki_1, kd_1, max_1, min_1)
  println("")
  println("")

  println("2nd config:")
  @printf("kp: %f, ki: %f, Kd: %f, max: %f, min: %f", kp_2, ki_2, kd_2, max_2, min_2)

  # Plot out
  t = 1:count

  p1 = Plots.plot(t, input_1, label="Input-1")
  Plots.plot!(t, input_2, label="Input-2")
  Plots.plot!(t, setPoint_1, label="Setpoint", linestyle=:dash)

  p2 = Plots.plot(t, output_1, label="Output-1", ls=:dash)
  Plots.plot!(t, iTerm_1, label="I Term-1")
  Plots.plot!(t, output_2, label="Output-2", ls=:dash)
  Plots.plot!(t, iTerm_2, label="I Term-2")

  Plots.display(Plots.plot(p1, p2, layout=(2, 1)))
end

end # module
