
include("pid_compute.jl")

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

