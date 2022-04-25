# PID Simulator

A simple PID controller simulator.

## Usage

1. Start Julia REPL.
2. Change the working dircetory to this folder:
```julia-repl
julia> cd("PATH_TO_THIS_FOLDER")
```

3. Activate environment:

```julia-repl
julia> ]
() pkg> activate .
  Activating environment at 'PATH_OF_ENV'
(PidSimulator) pkg>
```

4. Using `PidSimulator` module:
```julia-repl
julia> using PidSimulator
```

### Demo

Run `demo()` function:
```julia-repl
julia> demo()
--- PID Simulator Demo --
1st config:
kp: 1.500000, ki: 0.200000, Kd: 0.800000, max: 25.000000, min: -25.000000

2nd config:
kp: 2.500000, ki: 0.040000, Kd: 0.050000, max: 25.000000, min: -25.000000
```

![](https://i.imgur.com/sStyDpe.png)

### Plot

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