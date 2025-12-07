local u = {}

-- vector functions
local function vec3(x,y,z) return {x=x,y=y,z=z} end
local function vec3New() return {x=0,y=0,z=0} end
local function vec3Add(a,b) return vec3(a.x+b.x,a.y+b.y,a.z+b.z) end
local function vec3Sub(a,b) return vec3(a.x-b.x,a.y-b.y,a.z-b.z) end
local function vec3Scale(v,s) return vec3(v.x*s,v.y*s,v.z*s) end
local function vec3Mag(v) return math.sqrt((v.x*v.x)+(v.y*v.y)+(v.z*v.z)) end
local function vec3HMag(v) return math.sqrt((v.x*v.x)+(v.z*v.z)) end

local function vec3Norm(v)
    local mag = vec3Mag(v)
    if mag == 0 then return vec3(0,0,0) end
    return vec3Scale(v,1/mag)
end


local function vec3AimAt(v)
    local yaw = math.atan2(v.x,v.z)
    local pitch = math.atan2(v.y,math.sqrt(v.x^2+v.z^2))
    return pitch,yaw
end


-- works for minecraft coordinate system
local function vec3AimAtMC(v)
    -- minecraft yaw: 0° = +Z, +90° = -X, increases clockwise
    local yaw = math.atan2(-v.x, v.z)
    -- minecraft pitch: 0° = level, +up, -down
    local pitch = -math.atan2(v.y, math.sqrt(v.x*v.x + v.z*v.z))
    return pitch, yaw
end


local function clamp(n,min,max)
    if n < min then return min end
    if n > max then return max end
    return n
end


local function vec3Clamp(v,min,max)
    local vx = clamp(v.x,min,max)
    local vy = clamp(v.y,min,max)
    local vz = clamp(v.z,min,max)
    return vec3(vx,vy,vz)
end


local function vec3Dot(a,b)
    return a.x*b.x + a.y*b.y + a.z*b.z
end


local function vec3Cross(a,b)
    return u.vec3(
        a.y*b.z - a.z*b.y,
        a.z*b.x - a.x*b.z,
        a.x*b.y - a.y*b.x
    )
end


local function wrapAngle(theta)
    return (theta+math.pi)%(2*math.pi)-math.pi
end

local function wrapAngleDeg(theta)
    return math.deg(wrapAngle(math.rad(theta)))
end


local function lowPassFilter(err,prev_err,alpha)
    return prev_err * alpha + err * (1-alpha)
end


local function trun(n,sf)
    if sf == nil then sf = 3 end
    local f = 10^sf
    return math.floor(n*f)/f
end


local function printTrunVec(str,v)
    print(str,trun(v.x),trun(v.y),trun(v.z))
end


-- quaternion functions
-- for normalised quaternions only
local function quat(w,x,y,z)
    return {w=w,x=x,y=y,z=z}
end


local function quatNew()
    return quat(1,0,0,0)
end


local function quatConj(q)
    return {w=q.w,x=-q.x,y=-q.y,z=-q.z}
end


local function quatMul(a,b)
    return quat(
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    )
end


local function quatMag(q)
    return math.sqrt((q.w*q.w) + (q.x*q.x) + (q.y*q.y) + (q.z*q.z))
end


local function quatNorm(q)
    local mag = quatMag(q)
    if mag == 0 then return quat(1,0,0,0) end
    return quat(q.w/mag,q.x/mag,q.y/mag,q.z/mag)
end


-- not my function
local function axisAngleToQuat(axis,angle)
    axis = vec3Norm(axis)
    if vec3Mag(axis) == 0 or angle == 0 then
        return quat(1,0,0,0)
    end
    local ha = angle * 0.5
    local s = math.sin(ha)
    return quat(math.cos(ha),axis.x*s,axis.y*s,axis.z*s)
end


-- not my function
local function quatToAxisAngle(qi)
    local q = quatNorm(qi)
    local qw,qx,qy,qz=q.w,q.x,q.y,q.z

    if qw > 1 then qw = 1 end
    if qw < -1 then qw = -1 end

    local angle = 2 * math.acos(qw)
    local s = math.sqrt(1 - qw*qw)

    local axis
    if s < 1e-6 then
        axis = vec3(1,0,0)
    else
        axis = vec3(qx/s,qy/s,qz/s)
    end

    if angle > math.pi then
        angle = angle - 2*math.pi
        axis = vec3Scale(axis,-1)
    end

    return axis,angle
end


local function vec3RotateByQuat(v,q)
    local vq = quat(0,v.x,v.y,v.z)
    local qvq = quatMul(q,vq)
    local res = quatMul(qvq,quatConj(q))
    return vec3(res.x,res.y,res.z)
end


-- vs ship functions

local function shipGetPosition()
    local r = ship.getWorldspacePosition()
    return vec3(r.x,r.y,r.z)
end


local function shipGetQuat()
    local r = ship.getQuaternion()
    return quat(r.w,r.x,r.y,r.z)
end


local function shipGetQuatConj()
    local q = u.quatNorm(shipGetQuat())
    return quatConj(q)
end


local function vec3ToShipFrame(v)
    local qc = shipGetQuatConj()
    local vr = vec3RotateByQuat(v,qc)
    return vr
end


local function vec3DistVecToShip(v)
    local p = shipGetPosition()
    return vec3Sub(v,p)
end


local function vec3DistToShip(v)
    local p = shipGetPosition()
    local diff = vec3Sub(p,v)
    return vec3Mag(diff)
end


local function shipGetOmega()
    local r = ship.getOmega()
    return vec3(r.x,r.y,r.z)
end


local function shipGetLocalOmega()
    local r = shipGetOmega()
    return vec3ToShipFrame(r)
end


local function shipGetGlobalVelocity()
    local r = ship.getVelocity()
    return vec3(r.x,r.y,r.z)
end


local function shipGetLocalVelocity()
    return vec3ToShipFrame(shipGetGlobalVelocity())
end


local function shipGetSpeed()
    local v = shipGetGlobalVelocity()
    return vec3Mag(v)
end


local function shipGetAngularError(pos)
    local rel_pos = u.vec3DistVecToShip(pos)
    rel_pos = u.vec3ToShipFrame(rel_pos)
    local pitch,yaw = u.vec3AimAt(rel_pos)
    return math.sqrt((pitch*pitch)+(yaw*yaw))
end


u.vec3 = vec3
u.vec3New = vec3New
u.vec3Add = vec3Add
u.vec3AimAt = vec3AimAt
u.vec3AimAtMC = vec3AimAtMC
u.vec3DistToShip = vec3DistToShip
u.vec3DistVecToShip = vec3DistVecToShip
u.vec3Mag = vec3Mag
u.vec3HMag = vec3HMag
u.vec3Norm = vec3Norm
u.vec3RotateByQuat = vec3RotateByQuat
u.vec3Scale = vec3Scale
u.vec3Sub = vec3Sub
u.vec3ToShipFrame = vec3ToShipFrame
u.vec3Dot = vec3Dot
u.vec3Cross = vec3Cross

u.trun = trun
u.printTrunVec = printTrunVec
u.clamp = clamp
u.wrapAngle = wrapAngle
u.wrapAngleDeg = wrapAngleDeg
u.lowPassFilter = lowPassFilter
u.vec3Clamp = vec3Clamp

u.quat = quat
u.quatNew = quatNew
u.quatConj = quatConj
u.quatMag = quatMag
u.quatMul = quatMul
u.quatNorm = quatNorm
u.quatToAxisAngle = quatToAxisAngle
u.axisAngleToQuat = axisAngleToQuat

u.shipGetGlobalVelocity = shipGetGlobalVelocity
u.shipGetLocalOmega = shipGetLocalOmega
u.shipGetLocalVelocity = shipGetLocalVelocity
u.shipGetOmega = shipGetOmega
u.shipGetPosition = shipGetPosition
u.shipGetQuat = shipGetQuat
u.shipGetQuatConj = shipGetQuatConj
u.shipGetSpeed = shipGetSpeed
u.shipGetAngularError = shipGetAngularError

return u