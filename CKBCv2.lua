local u = require("mathUtilsCC")
local b = {}

-- project info: read documentation on github
-- developer: climbingkid_207#9008 (discord), XKNIGHTSEDGEX (minecraft username)


local function getDisplacement(v,a,dt)
    return u.vec3Add(u.vec3Scale(v,dt),u.vec3Scale(a,dt*dt*0.5))
end


local function getCannonState(is_ship,pos,barrel_len,proj_grav,drag_const,proj_muzzle_speed,mount_pitch,mount_yaw)
    local state = {}

    state.is_ship = is_ship
    state.barrel_len = barrel_len
    state.proj_grav = proj_grav
    state.drag_const = drag_const
    state.proj_muzzle_speed = proj_muzzle_speed
    state.cannon_mount_pitch = mount_pitch
    state.cannon_mount_yaw = mount_yaw

    if is_ship then
        state.mount_pos_offset = pos
        state.quatConj = u.shipGetQuatConj()
        state.velo = u.shipGetGlobalVelocity()
        state.pos = u.vec3Add(ship.shipGetPosition(),
        u.vec3RotateByQuat(cannonState.mount_pos_offset,u.quatNorm(u.shipGetQuat())))
    else
        state.mount_pos_offset = u.vec3New()
        state.quatConj = u.quatNew()
        state.velo = u.vec3New()
        state.pos = pos
    end
    return state
end


local function updateCannonState(state,mount_pitch,mount_yaw)
    state.cannon_mount_pitch = mount_pitch
    state.cannon_mount_yaw = mount_yaw
    if state.is_ship then
        state.quatConj = u.shipGetQuatConj()
        state.velo = u.shipGetGlobalVelocity()
        state.pos = ship.shipGetPosition()
    end
    return state
end


local function getTargetState(pos,velo,accel)
    local state = {}
    state.pos = pos
    state.velo = velo
    state.accel = accel
    return state
end


-- d: data table
local function simulateProjectile(d)
    -- simulation relative to cannon mount reference frame

    local sinp = math.sin(d.cannon_mount_pitch)
    local siny = math.sin(d.cannon_mount_yaw)

    local cosp = math.cos(d.cannon_mount_pitch)
    local cosy = math.cos(d.cannon_mount_yaw)

    local proj_velo_h = d.proj_muzzle_speed * cosp
    local proj_velo = u.vec3(proj_velo_h*siny,d.proj_muzzle_speed*sinp,proj_velo_h*cosy)

    -- projectile init at the end of the cannon barrel
    local proj_pos_h = d.cannon_barrel_len * cosp
    local proj_pos = u.vec3(proj_pos_h*siny,d.cannon_barrel_len*sinp,proj_pos_h*cosy)
    
    local t = 0
    local dt1 = 0.05
    local sim_TargetPos = u.vec3New()

    local prev_ptDist = math.huge
    local ptDist
    local ptDist_increase_count = 0

    while t <= d.break_time do
        proj_velo = u.vec3Add(proj_velo,u.vec3Scale(d.grav,dt1))
        -- drag_const is technically a damping factor (how drag is mimicked in CBC)
        proj_velo = u.vec3Scale(proj_velo,d.drag_const)
        proj_pos = u.vec3Add(proj_pos,proj_velo)

        sim_TargetPos = u.vec3Add(d.target_pos,getDisplacement(d.target_velo,d.target_accel,t))

        ptDist = u.vec3Mag(u.vec3Sub(proj_pos,sim_TargetPos))
        if ptDist > prev_ptDist then
            ptDist_increase_count = ptDist_increase_count + 1
            if ptDist_increase_count > 3 then break end
        else
            ptDist_increase_count = 0
        end
        prev_ptDist = ptDist

        t = t + dt1
    end

    -- yaw and pitch components of vectors, not to be confused with optimal cannon angles
    local pitch_Proj,yaw_Proj = u.vec3AimAt(proj_pos)
    local pitch_Target,yaw_Target = u.vec3AimAt(sim_TargetPos)

    -- low error doesn't guarantee interception just closest possible approach to target
    local yaw_error = u.wrapAngle(yaw_Target-yaw_Proj)
    local pitch_error = u.wrapAngle(pitch_Target-pitch_Proj)
    local proj_dist = u.vec3Mag(u.vec3Sub(sim_TargetPos,proj_pos))

    return pitch_error,yaw_error,proj_dist,proj_pos
end


-- returns a pitch and yaw RPM to hit a target
local function ballisticSolver(cannonState,targetState,convergence_sens)
    local glob_rel_target_pos = u.vec3Sub(targetState.pos,cannonState.pos)
    local losPitch,losYaw = u.vec3AimAt(glob_rel_target_pos)

    local simData = 
    {
    target_pos = u.vec3RotateByQuat(glob_rel_target_pos,cannonState.quatConj),  -- m vec
    target_velo = u.vec3RotateByQuat(targetState.velo,cannonState.quatConj),    -- m/s vec
    target_accel = u.vec3RotateByQuat(targetState.accel,cannonState.quatConj),  -- m/s^2 vec
    cannon_pos = cannonState.pos,                                               -- m vec
    cannon_velo = u.vec3RotateByQuat(cannonState.velo,cannonState.quatConj),    -- m/s vec
    cannon_barrel_len = cannonState.barrel_len,                                 -- m scalar
    grav = u.vec3RotateByQuat(cannonState.proj_grav,cannonState.quatConj),      -- m/tick^2 vec
    drag_const = cannonState.drag_const,                                        -- 0<=n<=1 scalar
    proj_muzzle_speed = cannonState.proj_muzzle_speed,                          -- m/tick scalar
    -- cannon mount angles are independant variables for the simulation not the actual in-game angles
    cannon_mount_pitch = losPitch,
    cannon_mount_yaw = losYaw,
    dt = 0.05,
    break_time = 5
    }

    local pitch_error,yaw_error,proj_dist,intercept_pos

    for i=1,10 do
        pitch_error,yaw_error,proj_dist,intercept_pos = simulateProjectile(simData)
        simData.cannon_mount_pitch = simData.cannon_mount_pitch + pitch_error * convergence_sens
        simData.cannon_mount_yaw = simData.cannon_mount_yaw + yaw_error * convergence_sens
    end

    print("ERR:",u.trun(math.sqrt(math.deg(pitch_error)^2+math.deg(yaw_error))^2))

    return simData.cannon_mount_pitch,simData.cannon_mount_yaw,proj_dist
end


local function getRPMCommand(cannonState,intercept_pitch,intercept_yaw,prev_intercept_pitch,prev_intercept_yaw,
                             prev_pitch_error,prev_yaw_error,c_dt,s_dt,c_kp,c_kd,s_kp,alpha,d_deadzone)
    local pitch_error = u.wrapAngle(intercept_pitch-cannonState.cannon_mount_pitch)
    local yaw_error = u.wrapAngle(intercept_yaw-cannonState.cannon_mount_yaw)

    --pitch_error = u.lowPassFilter(pitch_error,prev_pitch_error,alpha)
    --yaw_error = u.lowPassFilter(yaw_error,prev_yaw_error,alpha)

    local converge_pitch_P = pitch_error * c_kp
    local converge_pitch_D = (pitch_error-prev_pitch_error)*(c_kd/c_dt)
    if converge_pitch_D < d_deadzone then converge_pitch_D = 0 end
    local converge_pitch = converge_pitch_P + converge_pitch_D

    local converge_yaw_P = yaw_error * c_kp
    local converge_yaw_D = (yaw_error-prev_yaw_error)*(c_kd/c_dt)
    if converge_yaw_D < d_deadzone then converge_yaw_D = 0 end
    local converge_yaw = converge_yaw_P + converge_yaw_D

    local slew_rate_pitch = (intercept_pitch - prev_intercept_pitch)*(s_kp/s_dt)
    local slew_rate_yaw = (intercept_yaw - prev_intercept_yaw)*(s_kp/s_dt)

    local pitch_rpm = converge_pitch + slew_rate_pitch
    local yaw_rpm = converge_yaw + slew_rate_yaw

    return pitch_rpm,yaw_rpm,pitch_error,yaw_error
end

b.getDisplacement = getDisplacement
b.getCannonState = getCannonState
b.updateCannonState = updateCannonState
b.getTargetState = getTargetState
b.ballisticSolver = ballisticSolver
b.simulateProjectile = simulateProjectile
b.getRPMCommand = getRPMCommand

return b