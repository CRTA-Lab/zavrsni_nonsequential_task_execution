MODE = 'auto' -- 'sync', 'track' ili 'auto'
AUTO_TRACK_DELAY = 3.0

READY_SIGNAL = 'robot2_ready_for_robot1_run'
READY_DELAY_AFTER_REACH = 2

sim = require 'sim'
simIK = require 'simIK'
simROS2 = require('simROS2')

latestPositions = nil
sub = nil
pubState = nil

proxyTarget = nil
ikEnv = nil
ikGroup = nil

simBase = nil
simTip = nil
realTarget = nil
simJoints = {}
rosJointNames = {}

trackInitialized = false
autoSyncStartTime = nil

trackState = 'approach'
pathReadySent = false
pathReachTime = nil

lastPos = nil
lastVel = nil
filteredVel = nil
filteredAcc = nil

dx = 0
dy = 0
dz = 0

VEL_FILTER_ALPHA = 0.30
ACC_FILTER_ALPHA = 0.20

MAX_PUBLISH_VEL = 8
MAX_PUBLISH_ACC = 8

approachLinSpeed = 0.01
approachAngSpeed = 0.015

pathLinSpeed = 1.2
pathAngSpeed = 1.2

arrivalThreshold = 0.005


function wrapToPi(a)
    while a > math.pi do
        a = a - 2 * math.pi
    end
    while a < -math.pi do
        a = a + 2 * math.pi
    end
    return a
end


function clamp(x, limit)
    if x > limit then
        return limit
    end

    if x < -limit then
        return -limit
    end

    return x
end


function jointStateCb(msg)
    local nameToPos = {}

    for i = 1, #msg.name do
        nameToPos[msg.name[i]] = msg.position[i]
    end

    latestPositions = {
        nameToPos['robot2_shoulder_pan_joint'],
        nameToPos['robot2_shoulder_lift_joint'],
        nameToPos['robot2_elbow_joint'],
        nameToPos['robot2_wrist_1_joint'],
        nameToPos['robot2_wrist_2_joint'],
        nameToPos['robot2_wrist_3_joint']
    }
end


function applySync()
    if latestPositions ~= nil then
        for i = 1, 6 do
            if latestPositions[i] ~= nil then
                sim.setJointTargetPosition(simJoints[i], latestPositions[i])
                sim.setJointPosition(simJoints[i], latestPositions[i])
            end
        end
    end
end


function initDerivativeMemory()
    lastPos = {}
    lastVel = {}
    filteredVel = {}
    filteredAcc = {}

    for i = 1, #simJoints do
        lastPos[i] = sim.getJointPosition(simJoints[i])
        lastVel[i] = 0.0
        filteredVel[i] = 0.0
        filteredAcc[i] = 0.0
    end
end


function setupTrack()
    if trackInitialized then
        return
    end

    if sub then
        simROS2.shutdownSubscription(sub)
        sub = nil
    end

    dx = 0
    dy = 0
    dz = 0

    proxyTarget = sim.createDummy(0.01)
    sim.setObjectAlias(proxyTarget, 'proxyTarget_robot2_track')

    local tipPos = sim.getObjectPosition(simTip, -1)
    local tipOri = sim.getObjectOrientation(simTip, -1)

    sim.setObjectPosition(proxyTarget, -1, tipPos)
    sim.setObjectOrientation(proxyTarget, -1, tipOri)

    trackState = 'approach'
    pathReadySent = false
    pathReachTime = nil

    pubState = simROS2.createPublisher(
        '/robot2/coppelia_joint_target',
        'sensor_msgs/msg/JointState'
    )

    ikEnv = simIK.createEnvironment()
    ikGroup = simIK.createGroup(ikEnv)

    simIK.addElementFromScene(
        ikEnv,
        ikGroup,
        simBase,
        simTip,
        proxyTarget,
        simIK.constraint_x +
        simIK.constraint_y +
        simIK.constraint_z +
        simIK.constraint_alpha_beta +
        simIK.constraint_gamma
    )

    initDerivativeMemory()

    trackInitialized = true

    sim.addLog(sim.verbosity_scriptinfos, 'Robot2 TRACK initialized: approach to path start')
end


function publishJointStateWithVelocityAndAcceleration()
    if pubState == nil then
        return
    end

    local dt = sim.getSimulationTimeStep()

    if dt <= 0 then
        dt = 0.0166667
    end

    local pos = {}
    local rawVel = {}
    local rawAcc = {}
    local vel = {}
    local acc = {}

    for i = 1, #simJoints do
        pos[i] = sim.getJointPosition(simJoints[i])
    end

    if lastPos == nil then
        initDerivativeMemory()
    end

    for i = 1, #simJoints do
        local dq = wrapToPi(pos[i] - lastPos[i])
        rawVel[i] = dq / dt
    end

    for i = 1, #simJoints do
        rawAcc[i] = (rawVel[i] - lastVel[i]) / dt
    end

    for i = 1, #simJoints do
        filteredVel[i] = VEL_FILTER_ALPHA * rawVel[i] + (1.0 - VEL_FILTER_ALPHA) * filteredVel[i]
        filteredAcc[i] = ACC_FILTER_ALPHA * rawAcc[i] + (1.0 - ACC_FILTER_ALPHA) * filteredAcc[i]

        vel[i] = clamp(filteredVel[i], MAX_PUBLISH_VEL)
        acc[i] = clamp(filteredAcc[i], MAX_PUBLISH_ACC)
    end

    simROS2.publish(pubState, {
        name = rosJointNames,
        position = pos,
        velocity = vel,
        effort = acc
    })

    for i = 1, #simJoints do
        lastPos[i] = pos[i]
        lastVel[i] = rawVel[i]
    end
end


function doTrack()
    local dt = sim.getSimulationTimeStep()

    local pGoalRaw = sim.getObjectPosition(realTarget, -1)

    local pGoal = {
        pGoalRaw[1] + dx,
        pGoalRaw[2] + dy,
        pGoalRaw[3] + dz
    }

    local pCur = sim.getObjectPosition(proxyTarget, -1)

    local dxp = pGoal[1] - pCur[1]
    local dyp = pGoal[2] - pCur[2]
    local dzp = pGoal[3] - pCur[3]

    local dist = math.sqrt(dxp * dxp + dyp * dyp + dzp * dzp)

    if trackState == 'approach' and dist < arrivalThreshold then
        trackState = 'path'

        if pathReachTime == nil then
            pathReachTime = sim.getSimulationTime()
            sim.addLog(sim.verbosity_scriptinfos, 'Robot2 reached path start')
        end
    end

    if trackState == 'path' and not pathReadySent then
        if sim.getSimulationTime() - pathReachTime >= READY_DELAY_AFTER_REACH then
            sim.setInt32Signal(READY_SIGNAL, 1)
            pathReadySent = true
            sim.addLog(sim.verbosity_scriptinfos, 'Robot2 READY SIGNAL SENT -> robot1 may start')
        end
    end

    local maxLinSpeed = approachLinSpeed
    local maxAngSpeed = approachAngSpeed

    if trackState == 'path' then
        maxLinSpeed = pathLinSpeed
        maxAngSpeed = pathAngSpeed
    end

    local maxLinStep = maxLinSpeed * dt
    local pNew = {pCur[1], pCur[2], pCur[3]}

    if dist > 1e-6 then
        if dist <= maxLinStep then
            pNew = pGoal
        else
            local s = maxLinStep / dist

            pNew = {
                pCur[1] + dxp * s,
                pCur[2] + dyp * s,
                pCur[3] + dzp * s
            }
        end
    end

    sim.setObjectPosition(proxyTarget, -1, pNew)

    local oCur = sim.getObjectOrientation(proxyTarget, -1)
    local oGoal = sim.getObjectOrientation(realTarget, -1)

    local maxAngStep = maxAngSpeed * dt
    local oNew = {oCur[1], oCur[2], oCur[3]}

    for i = 1, 3 do
        local d = wrapToPi(oGoal[i] - oCur[i])

        if math.abs(d) <= maxAngStep then
            oNew[i] = oCur[i] + d
        else
            oNew[i] = oCur[i] + maxAngStep * (d / math.abs(d))
        end
    end

    sim.setObjectOrientation(proxyTarget, -1, oNew)

    simIK.handleGroup(ikEnv, ikGroup, {syncWorlds = true})

    publishJointStateWithVelocityAndAcceleration()
end


function sysCall_init()
    sim.setInt32Signal(READY_SIGNAL, 0)

    simBase = sim.getObject('/UR3e1')
    simTip = sim.getObject('/UR3e1/tool0_visual/tcp_tip')
    realTarget = sim.getObject('/UR3e2/tool0_visual/target2')

    simJoints = {
        sim.getObject('/UR3e1/Shoulder_pan_joint'),
        sim.getObject('/UR3e1/Shoulder_lift_joint'),
        sim.getObject('/UR3e1/Elbow_joint'),
        sim.getObject('/UR3e1/Wrist_1_joint'),
        sim.getObject('/UR3e1/Wrist_2_joint'),
        sim.getObject('/UR3e1/Wrist_3_joint')
    }

    rosJointNames = {
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'wrist_2_joint',
        'wrist_3_joint'
    }

    if MODE == 'sync' then
        sub = simROS2.createSubscription(
            '/robot2/joint_states',
            'sensor_msgs/msg/JointState',
            'jointStateCb'
        )

        sim.addLog(sim.verbosity_scriptinfos, 'MODE=sync robot2')

    elseif MODE == 'track' then
        setupTrack()
        sim.addLog(sim.verbosity_scriptinfos, 'MODE=track robot2')

    elseif MODE == 'auto' then
        sub = simROS2.createSubscription(
            '/robot2/joint_states',
            'sensor_msgs/msg/JointState',
            'jointStateCb'
        )

        sim.addLog(sim.verbosity_scriptinfos, 'MODE=auto robot2: sync first, then approach path start')

    else
        sim.addLog(sim.verbosity_scripterrors, 'Invalid MODE robot2')
    end
end


function sysCall_actuation()
    if MODE == 'sync' then
        applySync()

    elseif MODE == 'auto' then
        applySync()

        if latestPositions ~= nil then
            if autoSyncStartTime == nil then
                autoSyncStartTime = sim.getSimulationTime()
                sim.addLog(sim.verbosity_scriptinfos, 'Robot2 first joint_states received')
            end

            if sim.getSimulationTime() - autoSyncStartTime >= AUTO_TRACK_DELAY then
                setupTrack()
                MODE = 'track'
                sim.addLog(sim.verbosity_scriptinfos, 'AUTO switched robot2 from sync to track')
            end
        end

    elseif MODE == 'track' then
        if not trackInitialized then
            setupTrack()
        end

        doTrack()
    end
end


function sysCall_cleanup()
    sim.setInt32Signal(READY_SIGNAL, 0)

    if sub then
        simROS2.shutdownSubscription(sub)
    end

    if pubState then
        simROS2.shutdownPublisher(pubState)
    end

    if ikEnv then
        simIK.eraseEnvironment(ikEnv)
    end

    if proxyTarget then
        sim.removeObject(proxyTarget)
    end
end
