using JuMP
using Ipopt
using PyCall
PyDict(pyimport("matplotlib")["rcParams"])["font.sans-serif"] = ["Source Han Sans CN"]
using CSV, DataFrames
using PyPlot
using JSON

params = JSON.parsefile(string(@__DIR__, "/params.json"))
μ = params["miu"]
acc_comfort_max = params["acc_comfort_max"]
acc_comfort_min = params["acc_comfort_min"]
acc_engine_max = params["acc_engine_max"]
acc_engine_min = params["acc_engine_min"]
g = params["g"]
Cw = params["Cw"]
ω1 = 0.3
ω2 = 0.7

data_path = string(@__DIR__, "/../../../cache/")
input_file = string(params["file_prefix"],"_initial_speed.csv")
df = CSV.read(string(data_path, input_file), DataFrame; header=true)
s = Array(df[!,:s])
v0 = Array(df[!,:v0])
v_slide  = Array(df[!,:v_slide])
v_stairsend = Array(df[!,:vel_stairsend])
v_upper = Array(df[!,:v_upper])
a0 = Array(df[!,:a0])
a_slide = Array(df[!,:acc_slide])
a_rollover = Array(df[!,:acc_rollover])
a_upper = Array(df[!,:acc_upper])

n = length(s)

mdl = Model(Ipopt.Optimizer)
@variable(mdl, α[i=1:n])
@variable(mdl, β[i=1:n])

# initial value:
for i = 1:n
    set_start_value(β[i], v0[i]^2)
    if i < n
        set_start_value(α[i], (v0[i + 1]^2 - v0[i]^2) / 2.0 / (s[i + 1] - s[i]))
    end
    set_upper_bound(β[i], v0[i]^2)
    set_lower_bound(β[i], 0)
    set_upper_bound(α[i], a_upper[i])
    set_lower_bound(α[i], acc_comfort_min)
end

# constraints:
for i = 1:n
    # dynamic constraint:
    if i < n
        @constraint(mdl, α[i] == (β[i + 1] - β[i]) / (s[i + 1] - s[i]) / 2.0)
    else
        @constraint(mdl, α[n] == (β[n] - β[n - 1]) / (s[n] - s[n - 1]) / 2.0)
    end
end
# bounds constraint
@constraint(mdl, β[1] == v0[1]^2)
@constraint(mdl, β[n] == v0[n]^2)
@constraint(mdl, α[n] == 0)
# @constraint(mdl, α[n] == 0)

# objective
@NLobjective(mdl,
    Min,
    2 * ω1 * sum((s[i + 1] - s[i]) / (sqrt(β[i + 1] + 1e-1) + sqrt(β[i] + 1e-1) + 1e-1) for i = 1:n - 1) + 
    ω2 * sum(((α[i + 1] - α[i]) / (s[i + 1] - s[i]))^2 * (s[i + 1] - s[i]) for i = 1:n - 1)
    )

@time status = optimize!(mdl)

######################## plot #######################
if params["is_plot"]
    # vel = value.(β)
    # vel = sqrt.(vel)
    # acc = value.(α)
    # s_new = s[1:n]
    # pyplot()
    # plot(s_new,vel,linecolor=:green,linestyle = :solid,
    #     xlabel="弧长 (m)",ylabel="速度 (m/s)",label="优化后速度曲线",
    #     legendfont="KaiTi",tickfont="KaiTi",guidefont="KaiTi")
    # plot!(s_new,v0,linecolor=:blue,linestyle = :solid,
    #     label="约束后速度曲线",
    #     legendfont="KaiTi",tickfont="KaiTi",guidefont="KaiTi")
    # plot!(s_new,v_slide,linecolor=:red,linestyle = :dash,
    #     label="曲率约束",
    #     legendfont="KaiTi",tickfont="KaiTi",guidefont="KaiTi")
    # plot!(s_new,v_stairsend,linecolor=:yellow,linestyle = :dash,
    #     label="冲击约束",
    #     legendfont="KaiTi",tickfont="KaiTi",guidefont="KaiTi")

    # plot(s_new,acc,linecolor=:green,linestyle = :solid,
    #     xlabel="弧长 (m)",ylabel="加速度 (m/s^2)",label="优化后加速度曲线",
    #     legendfont="KaiTi",tickfont="KaiTi",guidefont="KaiTi")
    # plot!(s_new,a0,linecolor=:blue,linestyle = :solid,
    #     label="约束后加速度曲线",
    #     legendfont="KaiTi",tickfont="KaiTi",guidefont="KaiTi")
    # plot!(s_new,a_slide,linecolor=:red,linestyle = :dash,
    #     label="驱动约束",
    #     legendfont="KaiTi",tickfont="KaiTi",guidefont="KaiTi")
    # plot!(s_new,v_stairsend,linecolor=:yellow,linestyle = :dash,
    #     label="倾翻约束",
    #     legendfont="KaiTi",tickfont="KaiTi",guidefont=""KaiTi")
    #show()
    pygui(true)
    vel = value.(β)
    vel = sqrt.(vel)
    acc = value.(α)
    s_new = s[1:n]

    figure("velocity")
    plot(s_new, vel, "g", s, v0, "b", s, v_slide, "r--", s, v_stairsend, "y--")
    grid(true)
    legend(["优化后速度曲线", "优化前速度曲线", "曲率约束", "冲击约束"],loc=0)
    ylabel("速度 (m/s)")
    xlabel("弧长 (m)")
    figure("accelation")
    plot(s_new, acc, "g", s, a0, "b", s, a_slide, "r--", s, a_rollover, "y--")
    grid(true)
    legend(["优化后加速度曲线", "优化前加速度曲线", "驱动约束", "倾翻约束"],loc=0)
    ylabel("加速度 (m/s^2)")
    xlabel("弧长 (m)")
    show()
end

######################## save in file ###################
vel = value.(β)
vel = sqrt.(vel)
acc = value.(α)
df.vel = vel
df.acc = acc
output_file = string(params["file_prefix"],"_optimized_speed.csv")

CSV.write(string(data_path, output_file), df)