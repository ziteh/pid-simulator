function pid_compute(
  set_value, actual_value, kp, ki, kd, i_term_prev, error_prev, iteration_time=1.0;
  max=typemax(Float64), min=typemin(Float64), bias=0.0)

  error = set_value - actual_value

  p_term = kp * error
  i_term = ki * (i_term_prev + error * iteration_time)
  d_term = kd * (error - error_prev) / iteration_time

  # if i_term > max
  #   i_term = max
  # elseif i_term < min
  #   i_term = min
  # end

  output = p_term + i_term + d_term + bias

  if output > max
    output = max
  elseif output < min
    output = min
  end

  return (output, i_term, error)
end