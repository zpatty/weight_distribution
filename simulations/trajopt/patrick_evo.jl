using Pkg
Pkg.activate(joinpath(@__DIR__, "../examples/"))
Pkg.add("MAT")
Pkg.instantiate()

# ## Setup
using Dojo
using Random
using LinearAlgebra 
using JLD2
using StaticArrays
using IterativeLQR
import Dojo: cost
using Dates
using MAT
using DelimitedFiles
using Evolutionary

#include(joinpath(@__DIR__, "algorithms/ags.jl")) # augmented random search
include(joinpath(@__DIR__, "../environments/patrick/methods/initialize.jl"))
include(joinpath(@__DIR__, "../environments/patrick/methods/env.jl"))
# include("patrick_struct.jl")


mass_in_limbs = 0.75
mass_proximal = 0.75
mass_total = 0.33
limb_length = 0.13

num_limbs = 5 # wouldn't recommend doing less than 3 for now. lemme know if you want less
num_links_per_limb = 2
num_actuated_per_limb = 2

radius_b = 0.045
height = 0.032
body_position=[0.0; 0.0; height/2 + height/100]

gravity=[0.0, 0.0, -2.81]
timestep=0.05

T = 51

model = Patrick([mass_total, limb_length, mass_proximal, mass_in_limbs]);

# println("mass_proximal = $k, mass_in_limbs = $l")
obs = reset(model.env);
# environments[m].state .= get_minimal_state(environments[m].mechanism);
# render(environments[m]);



function fitness(model)
    n = model.env.num_states
    m = model.env.num_inputs

    @load joinpath(@__DIR__, "results/patrick_ilqr-2022-06-18 14:22:13.jld2") x_sol u_sol
    ū = [zeros(model.env.num_inputs) for i in 1:T-1]
    u_sol_start = u_sol;
    ū .= u_sol_start;



    xref = [[-1.0*t/25; 0.0; height; zeros(3); -0.5; zeros(5); x_sol[t+1][13:end]] for t=0:T-1]
    # xref .= x_sol_old
    timestep=0.05

    relative_input_cost = 420000 #50
    qt = [50.0; 0.0; 10000; zeros(3); 10.0; zeros(5); zeros(length(model.env.state)-12)]
    # qt = [50*ones(12); 50*ones(length(env.state)-12)]
    qt_end = [100.0; 0.0; 100; 1.0*ones(3); 10.0; zeros(5); 100*ones(length(model.env.state)-12)]
    # qt_end = [50*ones(12); 50*ones(length(env.state)-12)]
    rt = repeat([1; 1; 1000; 1000], num_limbs)
    ots = [(x, u, w) -> transpose(x - xref[t]) * Diagonal(timestep * qt) * (x - xref[t]) + relative_input_cost * transpose(u) * Diagonal(timestep * ones(m)) * u for t = 1:T-1]
    oT = (x, u, w) -> 5*transpose(x - xref[end]) * Diagonal(timestep * qt_end) * (x - xref[end])

    function goal(x, u, w)
        Δ = x - xref
        return Δ[collect(1:3)]
    end

    dyn = IterativeLQR.Dynamics(
    (y, x, u, w) -> dynamics(y, model.env, x, u, w),
    (dx, x, u, w) -> dynamics_jacobian_state(dx, model.env, x, u, w),
    (du, x, u, w) -> dynamics_jacobian_input(du, model.env, x, u, w),
    n, n, m)

    dyn_vec = [dyn for t = 1:T-1]

    # ## rollout
    initialize_patrick!(model.env.mechanism,body_position=body_position,body_orientation=[0.0; 0.0; 0.0 * π])
    x1 = get_minimal_state(model.env.mechanism);
    # x1 = x_sol[1];
    x̄ = IterativeLQR.rollout(dyn_vec, x1, ū)
    visualize(model.env, x̄);

    cts = [IterativeLQR.Cost(ot, n, m) for ot in ots]
    cT = IterativeLQR.Cost(oT, n, 0)
    obj = [cts..., cT]

    cont = IterativeLQR.Constraint()
    # conT = IterativeLQR.Constraint(goal, n, 0)
    cons = [[cont for t = 1:T-1]..., cont]

    # ## solver
    s = IterativeLQR.solver(dyn_vec, obj, cons, 
    opts=IterativeLQR.Options(
        max_al_iter=10,
        verbose=false))

    IterativeLQR.initialize_controls!(s, ū)
    IterativeLQR.initialize_states!(s, x̄)

    # ## solve
    # @time IterativeLQR.solve!(s)

    # ## solution
    x_sol, u_sol = IterativeLQR.get_trajectory(s)

    return x_sol[end][1]^2 * 10 - 100*sum([x[3]^2 for x in x_sol]) - sum([u'*u for u in u_sol])
end

function crossover(m1,m2)
    params1 = m1.params
    params2 = m2.params
    n1, n2 = AX(params1, params2)
    return Patrick(n1), Patrick(n2)
end

lower = zeros(4);
upper = [0.6, 0.25, 1.0, 1.0]
function uniform_pat(r = 0.5)
    vop = uniform(r)
    function mutation(model::T) where {T <: Patrick}  
        θ = vop(zeros(4)) + ones(4) * 0.5
        # new_params = [0.33*2*θ[1], 0.13*2 * θ[2], θ[3], θ[4]]
        model.params .= [0.33*2*θ[1], 0.13*2 * θ[2], θ[3], θ[4]]
        return Patrick(model.params)
    end
    return mutation
end

import Evolutionary.initial_population
function initial_population(method::M, individual::Patrick;
                            kwargs...) where {M<:Evolutionary.AbstractOptimizer}
    params = [rand()*(0.33*2 - 0.33/2); rand()*(0.13*2 - 0.13/2); rand(); rand()]
    [Patrick(params) for i in 1:Evolutionary.population_size(method)]
end

import Evolutionary.EvolutionaryObjective
function EvolutionaryObjective(f, x::Patrick; eval::Symbol = :serial)
    fval = f(x)
    EvolutionaryObjective{typeof(f),typeof(fval),typeof(x),Val{eval}}(f, fval, deepcopy(x), 0)
end

import Base: copy

copy(pat::Patrick) = deepcopy(pat)