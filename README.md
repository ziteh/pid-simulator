# PID Simulator

A simple PID controller simulator.

The PID comput code in the [`pid_compute.jl`](/pid_compute.jl).

## Usage

1. Start Julia REPL.
2. Change the working dircetory to this folder:
```julia-repl
julia> cd("PATH_TO_THIS_FOLDER")
```
3. Check the current working directory:
```julia-repl
julia> pwd()
"CURRENT_WORKING_DIR"
```

### Demo

Run `pid_demo.jl`:
```julia-repl
julia> include("pid_demo.jl")
--- PID Simulator Demo ---
1st config:
kp: 0.400000, ki: 0.900000, Kd: 0.500000, max: 55.000000, min: -55.000000
2nd config:
kp: 1.500000, ki: 0.200000, Kd: 0.500000, max: 55.000000, min: -55.000000
Done!
```

![](https://i.imgur.com/BWwxOPX.png)

### Plot
Include `pid_plot.jl` first:
```julia-repl
julia> include("pid_plot.jl")
```

Run `plot()` function with specified `kp=1.5`, `ki=0.2` and `kd=0.3`:
```julia-repl
julia> plot(1.5, 0.2, 0.3)
```

Run `plot()` function with specified `kp`, `ki`, `kd` and `max=25`, `min=-25`:
```julia-repl
julia> plot(1.5, 0.2, 0.3, 25, -25)
```

Run `plot()` function with specified `kp`, `ki`, `kd`, `max`, `min` and `system_model`:
```julia-repl
julia> plot(1.5, 0.2, 0.3, 25, -25, system_model=(i,b)->i+b*0.7+5)
```

## Environment

- Julia `v1.6.6` LTS 64-bit
  - Plots