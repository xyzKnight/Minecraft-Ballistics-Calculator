local u = require("mathUtilsCC")
local b = require("CKBCv2")

-- SETTINGS

-- PD TUNING SETTINGS
local RPM_CONVERGE_KP = 300
local RPM_CONVERGE_KD = 25
local RPM_SLEW_KP = 100
local RPM_ALPHA = 0.5
local RPM_DERIV_DEADZONE = 0.01

-- PERIPHERALS
local MOUNT_READER = peripheral.wrap("back")
local RADAR = peripheral.wrap("bottom")
local PITCH_RSC = peripheral.wrap("left")
local YAW_RSC = peripheral.wrap("right")
local PLAYER_TRACKER = peripheral.wrap("front")

-- SYS
local RUNNING = true
local sys_dt = 0.05
local prev_time_sys = 0
local current_time_sys = os.clock()

local prev_pitch_error = 0
local prev_yaw_error = 0

local prev_solver_pitch = 0
local prev_solver_yaw = 0

-- RADAR

--local TARGET_ID = 39
local TARGET_ID = nil
local RADAR_RANGE = 1000
local ENGAGEMENT_RANGE = 500

-- comp is commanding player
local COMMANDING_PLAYER = "XKNIGHTSEDGEX"
local min_comp_view_error = math.rad(10)
local comp_pos = u.vec3New()
local comp_pitch = 0
local comp_yaw = 0

-- PROJECTILE STATE
local DRAG_DAMPER = 0.99
local MUZZLE_SPEED = 10
local GRAV = u.vec3(0,-0.5,0)

local MIN_MISS_DIST = 3
local BSOLVER_CONVERGENCE_SENS = 0.5

-- CANNON STATE
local cannon_mount_pitch = 0
local cannon_mount_yaw = 0
local BARREL_LEN = 6

--local CANNON_MOUNT_OFFSET = u.vec3New()
local CANNON_MOUNT_POS = u.vec3(234,-57,26)

local pitch_RPM = 0
local yaw_RPM = 0

--local cannonState = b.getCannonState(true,CANNON_MOUNT_OFFSET,BARREL_LEN,GRAV,DRAG_DAMPER,MUZZLE_SPEED,cannon_mount_pitch,cannon_mount_yaw)
local cannonState = b.getCannonState(false,CANNON_MOUNT_POS,BARREL_LEN,GRAV,DRAG_DAMPER,MUZZLE_SPEED,cannon_mount_pitch,cannon_mount_yaw)

-- TARGET STATE
local target_pos = u.vec3New()
local target_velo = u.vec3New()
local target_accel = u.vec3New()
local prev_raw_vel = u.vec3New()

local raw_vel_list = {}
local smooth_vel_list = {}
local smooth_accel_list = {}

local VEL_LIST_MAX_LEN = 50
local ACCEL_LIST_MAX_LEN = 3

-- must be odd
local VEL_WINDOW = 5
local ACC_WINDOW = 3

local prev_radar_call_time = 0
local target_vel_sample_dt = 0

local function smooth_vec_list(list, window)
    local half = (window-1)/2
    if #list < window then return list[#list] end
    local sum = u.vec3(0,0,0)
    for i = #list - window + 1, #list do
        sum = u.vec3Add(sum,list[i])
    end
    return u.vec3Scale(sum,1/window)
end


local function central_diff(prevv,nextv,dt)
    return u.vec3Scale(u.vec3Sub(nextv,prevv),1/(2*dt))
end

local function push_velocity(v)
    table.insert(raw_vel_list,v)
    if #raw_vel_list > VEL_LIST_MAX_LEN then table.remove(raw_vel_list,1) end

    local v_smooth = smooth_vec_list(raw_vel_list,VEL_WINDOW)
    table.insert(smooth_vel_list,v_smooth)
    if #smooth_vel_list > VEL_LIST_MAX_LEN then table.remove(smooth_vel_list,1) end
    return v_smooth
end


local function compute_accel()
    if #smooth_vel_list < VEL_WINDOW then return u.vec3New() end
    local n = #smooth_vel_list
    local v_prev = smooth_vel_list[n-2]
    local v_next = smooth_vel_list[n]
    local a_raw = central_diff(v_prev,v_next,target_vel_sample_dt)

    table.insert(smooth_accel_list,a_raw)
    if #smooth_accel_list > ACCEL_LIST_MAX_LEN then table.remove(smooth_accel_list,1) end
    return smooth_vec_list(smooth_accel_list,ACC_WINDOW)
end


local function update_target_ID(scanned_ships)
    if rs.getInput("front") == true then return end
    for i=1,#scanned_ships do
        local ship = scanned_ships[i]
        local rel_pos = u.vec3Sub(ship.pos,comp_pos)
        local pitchToShip,yawToShip = u.vec3AimAtMC(rel_pos)

        local comp_pitch_err = math.abs(u.wrapAngle(pitchToShip-comp_pitch))
        local comp_yaw_err = math.abs(u.wrapAngle(yawToShip-comp_yaw))
        --local view_angle_err = (comp_pitch_err + comp_yaw_err)/2
        local view_angle_err = comp_yaw_err 
        --local view_angle_err = comp_pitch_err
        
        print(math.deg(view_angle_err))

        if view_angle_err < min_comp_view_error then
            TARGET_ID = ship.id
            break
        end
    end
end


local function update_target_motion()
    while RUNNING do

        local hit_ship = nil
        local scanned_ships = RADAR.scanForShips(RADAR_RANGE)

        update_target_ID(scanned_ships)
        print(TARGET_ID)

        for i=1,#scanned_ships do
            if scanned_ships[i].id == TARGET_ID then hit_ship = scanned_ships[i]; break end
        end

        if hit_ship ~= nil then
            target_pos = hit_ship.pos

            target_velo = push_velocity(hit_ship.velocity)
                
            local time = os.clock()
            target_vel_sample_dt = time - prev_radar_call_time
            prev_radar_call_time = time

            target_accel = compute_accel()
            
        end
    end
end


local function update_comp_dir()
    while RUNNING do
        local comp_data = PLAYER_TRACKER.getPlayerPos(COMMANDING_PLAYER)
        local x = comp_data.x
        local y = comp_data.y
        local z = comp_data.z
        comp_pos = u.vec3(x,y,z)
        -- deg
        comp_pitch = -math.rad(comp_data.pitch)
        comp_yaw = math.rad(comp_data.yaw)
    end
end


local function main()
    --os.sleep(1)
    while RUNNING do
        term.clear()
        term.setCursorPos(1,1)

        current_time_sys = os.clock()
        sys_dt = current_time_sys-prev_time_sys
        prev_time_sys = current_time_sys
        if sys_dt == 0 then sys_dt = 0.05 end

        cannonState = b.updateCannonState(cannonState,cannon_mount_pitch,cannon_mount_yaw)
        targetState = b.getTargetState(target_pos,target_velo,target_accel)

        if TARGET_ID ~= nil then
            --u.printTrunVec("TP:",target_pos)
            --u.printTrunVec("TV:",target_velo)
            --u.printTrunVec("TA:",target_accel)

            local solver_pitch,solver_yaw,miss_dist = b.ballisticSolver(cannonState,targetState,BSOLVER_CONVERGENCE_SENS)
            --local solver_pitch,solver_yaw = u.vec3AimAt(u.vec3Sub(targetState.pos,cannonState.pos))
            --local miss_dist = 0
            solver_pitch = u.clamp(solver_pitch,0,100)
            solver_yaw = -solver_yaw
            
            -- pitch and yaw RPM globals are directly output to RSC
            pitch_RPM,yaw_RPM,prev_pitch_error,prev_yaw_error = b.getRPMCommand(cannonState,solver_pitch,solver_yaw,prev_solver_pitch,prev_solver_yaw,
                        prev_pitch_error,prev_yaw_error,sys_dt,sys_dt,RPM_CONVERGE_KP,RPM_CONVERGE_KD,RPM_SLEW_KP,RPM_ALPHA,RPM_DERIV_DEADZONE)
            prev_solver_pitch,prev_solver_yaw = solver_pitch,solver_yaw
            pitch_RPM = -pitch_RPM

            print("PRPM:"..u.trun(pitch_RPM))
            print("YRPM:"..u.trun(yaw_RPM))
            print("DT:",u.trun(sys_dt))

            u.printTrunVec("TPOS:",target_pos)
            u.printTrunVec("TVEL:",target_velo)
            u.printTrunVec("TACCEL:",target_accel)
        else
            pitch_RPM,yaw_RPM = 0,0
        end

        os.sleep(0)
    end
end


local function get_cannon_pitch_yaw()
    while RUNNING do
        local mount_data = MOUNT_READER.getBlockData()
        cannon_mount_pitch = math.rad(mount_data.CannonPitch)
        cannon_mount_yaw = math.rad(mount_data.CannonYaw)
    end
end

local function output_pitch()
    while RUNNING do
        PITCH_RSC.setTargetSpeed(pitch_RPM)
    end
end

local function output_yaw()
    while RUNNING do
        YAW_RSC.setTargetSpeed(yaw_RPM)
    end
end

parallel.waitForAll(main,update_target_motion,update_comp_dir,get_cannon_pitch_yaw,output_pitch,output_yaw)