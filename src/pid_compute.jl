function pid_compute(setpoint, input, kp, ki, kd, last_iterm, last_input, max=typemax(Float64), min=typemin(Float64))
  error = setpoint - input
  diff = input - last_input

  iterm = ki * error + last_iterm
  
  if iterm > max
    iterm = max
  elseif iterm < min
    iterm = min
  end

  output = kp * error + iterm - kd * diff

  if output > max
    output = max
  elseif output < min
    output = min
  end

  return (output, iterm, input)
end