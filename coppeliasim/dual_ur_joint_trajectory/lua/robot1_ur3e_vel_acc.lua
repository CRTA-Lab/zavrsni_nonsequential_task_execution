MODE = 'auto' -- 'sync', 'run' ili 'auto'
AUTO_RUN_DELAY = 3.0

READY_SIGNAL = 'robot2_ready_for_robot1_run'

sim = require 'sim'
simROS2 = require('simROS2')
simIK = require 'simIK'

latestPositions = nil
sub = nil
pubState = nil

proxyTarget = nil
ikEnv = nil
ikGroup = nil

simBase = nil
simTip = nil
simJoints = {}
rosJointNames = {}

targetNames = {}
targets = {}
currentTargetIndex = 1

lastPos = nil
lastVel = nil
filteredVel = nil
filteredAcc = nil

autoSyncStartTime = nil
runInitialized = false
waitingReadyLogPrinted = false

VEL_FILTER_ALPHA = 0.35
ACC_FILTER_ALPHA = 0.20

MAX_PUBLISH_VEL = 8.0
MAX_PUBLISH_ACC = 8.0

maxLinSpeed = 0.028
maxAngSpeed = 0.028
maxLinAccel = 0.09
maxAngAccel = 0.09

arrivalThreshold = 0.006
TARGET_DWELL_TIME = 0.15

proxyLinVel = {0.0, 0.0, 0.0}
proxyAngVel = {0.0, 0.0, 0.0}
targetDwellStartTime = nil
isDwelling = false


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


function vecNorm(v)
    return math.sqrt(v[1] * v[1] + v[2] * v[2] + v[3] * v[3])
end


function limitVector(v, maxMag)
    local n = vecNorm(v)

    if n > maxMag and n > 1e-9 then
        local s = maxMag / n

        return {
            v[1] * s,
            v[2] * s,
            v[3] * s
        }
    end

    return v
end


function approachVector(current, desired, maxDelta)
    local delta = {
        desired[1] - current[1],
        desired[2] - current[2],
        desired[3] - current[3]
    }

    delta = limitVector(delta, maxDelta)

    return {
        current[1] + delta[1],
        current[2] + delta[2],
        current[3] + delta[3]
    }
end


function jointStateCb(msg)
    local nameToPos = {}

    for i = 1, #msg.name do
        nameToPos[msg.name[i]] = msg.position[i]
    end

    latestPositions = {
        nameToPos['robot1_shoulder_pan_joint'],
        nameToPos['robot1_shoulder_lift_joint'],
        nameToPos['robot1_elbow_joint'],
        nameToPos['robot1_wrist_1_joint'],
        nameToPos['robot1_wrist_2_joint'],
        nameToPos['robot1_wrist_3_joint']
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


function resetProxyMotion()
    proxyLinVel = {0.0, 0.0, 0.0}
    proxyAngVel = {0.0, 0.0, 0.0}
    targetDwellStartTime = nil
    isDwelling = false
end


function goToNextTarget()
    currentTargetIndex = currentTargetIndex + 1

    if currentTargetIndex > #targets then
        currentTargetIndex = 1
    end

    targetDwellStartTime = nil
    isDwelling = false

    sim.addLog(sim.verbosity_scriptinfos, 'Robot1 next target: ' .. targetNames[currentTargetIndex])
end


function setupRun()
    if runInitialized then
        return
    end

    if sub then
        simROS2.shutdownSubscription(sub)
        sub = nil
    end

    pubState = simROS2.createPublisher(
        '/robot1/coppelia_joint_target',
        'sensor_msgs/msg/JointState'
    )

    targetNames = {
        '/target3',
        '/target4',
        '/target5',
        '/target6',
        '/target7',
        '/target8',
        '/target9',
        '/target10'
    }

    targets = {}

    for i = 1, #targetNames do
        targets[i] = sim.getObject(targetNames[i])
    end

    currentTargetIndex = 1

    proxyTarget = sim.createDummy(0.01)
    sim.setObjectAlias(proxyTarget, 'proxyTarget_robot1_joint_trajectory_wait_ready')

    local tipPos = sim.getObjectPosition(simTip, -1)
    local tipOri = sim.getObjectOrientation(simTip, -1)

    sim.setObjectPosition(proxyTarget, -1, tipPos)
    sim.setObjectOrientation(proxyTarget, -1, tipOri)

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
    resetProxyMotion()

    runInitialized = true

    sim.addLog(
        sim.verbosity_scriptinfos,
        'Robot1 RUN started after robot2 ready signal'
    )
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


function updateProxyPositionSoft(dt, currentTarget)
    local pGoal = sim.getObjectPosition(currentTarget, -1)
    local pCur = sim.getObjectPosition(proxyTarget, -1)

    local dx = pGoal[1] - pCur[1]
    local dy = pGoal[2] - pCur[2]
    local dz = pGoal[3] - pCur[3]

    local dist = math.sqrt(dx * dx + dy * dy + dz * dz)

    if dist < arrivalThreshold then
        sim.setObjectPosition(proxyTarget, -1, pGoal)

        proxyLinVel = {0.0, 0.0, 0.0}

        if not isDwelling then
            isDwelling = true
            targetDwellStartTime = sim.getSimulationTime()
        end

        if sim.getSimulationTime() - targetDwellStartTime >= TARGET_DWELL_TIME then
            goToNextTarget()
        end

        return
    end

    isDwelling = false
    targetDwellStartTime = nil

    local dir = {0.0, 0.0, 0.0}

    if dist > 1e-9 then
        dir = {
            dx / dist,
            dy / dist,
            dz / dist
        }
    end

    local desiredSpeed = math.sqrt(2.0 * maxLinAccel * dist)

    if desiredSpeed > maxLinSpeed then
        desiredSpeed = maxLinSpeed
    end

    local desiredVel = {
        dir[1] * desiredSpeed,
        dir[2] * desiredSpeed,
        dir[3] * desiredSpeed
    }

    proxyLinVel = approachVector(proxyLinVel, desiredVel, maxLinAccel * dt)

    local step = {
        proxyLinVel[1] * dt,
        proxyLinVel[2] * dt,
        proxyLinVel[3] * dt
    }

    local stepNorm = vecNorm(step)

    if stepNorm > dist then
        sim.setObjectPosition(proxyTarget, -1, pGoal)
        proxyLinVel = {0.0, 0.0, 0.0}
    else
        local pNew = {
            pCur[1] + step[1],
            pCur[2] + step[2],
            pCur[3] + step[3]
        }

        sim.setObjectPosition(proxyTarget, -1, pNew)
    end
end


function updateProxyOrientationSoft(dt, currentTarget)
    local oCur = sim.getObjectOrientation(proxyTarget, -1)
    local oGoal = sim.getObjectOrientation(currentTarget, -1)

    local err = {
        wrapToPi(oGoal[1] - oCur[1]),
        wrapToPi(oGoal[2] - oCur[2]),
        wrapToPi(oGoal[3] - oCur[3])
    }

    local errNorm = vecNorm(err)

    if errNorm < 0.003 then
        sim.setObjectOrientation(proxyTarget, -1, oGoal)
        proxyAngVel = {0.0, 0.0, 0.0}
        return
    end

    local dir = {
        err[1] / errNorm,
        err[2] / errNorm,
        err[3] / errNorm
    }

    local desiredAngSpeed = math.sqrt(2.0 * maxAngAccel * errNorm)

    if desiredAngSpeed > maxAngSpeed then
        desiredAngSpeed = maxAngSpeed
    end

    local desiredAngVel = {
        dir[1] * desiredAngSpeed,
        dir[2] * desiredAngSpeed,
        dir[3] * desiredAngSpeed
    }

    proxyAngVel = approachVector(proxyAngVel, desiredAngVel, maxAngAccel * dt)

    local oNew = {
        oCur[1] + proxyAngVel[1] * dt,
        oCur[2] + proxyAngVel[2] * dt,
        oCur[3] + proxyAngVel[3] * dt
    }

    sim.setObjectOrientation(proxyTarget, -1, oNew)
end


function doRun()
    local dt = sim.getSimulationTimeStep()

    if dt <= 0 then
        dt = 0.0166667
    end

    local currentTarget = targets[currentTargetIndex]

    updateProxyPositionSoft(dt, currentTarget)
    updateProxyOrientationSoft(dt, currentTarget)

    simIK.handleGroup(ikEnv, ikGroup, {syncWorlds = true})

    publishJointStateWithVelocityAndAcceleration()
end


function sysCall_init()
    simBase = sim.getObject('/UR3e2')
    simTip = sim.getObject('/UR3e2/tool0_visual/tcp_tip_2')

    simJoints = {
        sim.getObject('/UR3e2/shoulder_pan_joint'),
        sim.getObject('/UR3e2/shoulder_lift_joint'),
        sim.getObject('/UR3e2/elbow_joint'),
        sim.getObject('/UR3e2/wrist_1_joint'),
        sim.getObject('/UR3e2/wrist_2_joint'),
        sim.getObject('/UR3e2/wrist_3_joint')
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
            '/robot1/joint_states',
            'sensor_msgs/msg/JointState',
            'jointStateCb'
        )

        sim.addLog(sim.verbosity_scriptinfos, 'MODE=sync robot1')

    elseif MODE == 'run' then
        setupRun()
        sim.addLog(sim.verbosity_scriptinfos, 'MODE=run robot1')

    elseif MODE == 'auto' then
        sub = simROS2.createSubscription(
            '/robot1/joint_states',
            'sensor_msgs/msg/JointState',
            'jointStateCb'
        )

        sim.addLog(
            sim.verbosity_scriptinfos,
            'MODE=auto robot1: sync, then wait for robot2 ready signal'
        )

    else
        sim.addLog(sim.verbosity_scripterrors, 'Invalid MODE robot1')
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
                sim.addLog(sim.verbosity_scriptinfos, 'Robot1 first joint_states received, sync timer started')
            end

            local ready = sim.getInt32Signal(READY_SIGNAL)

            if sim.getSimulationTime() - autoSyncStartTime >= AUTO_RUN_DELAY then
                if ready == 1 then
                    setupRun()
                    MODE = 'run'
                    sim.addLog(sim.verbosity_scriptinfos, 'AUTO switched robot1 from sync to run after robot2 ready')
                else
                    if not waitingReadyLogPrinted then
                        sim.addLog(sim.verbosity_scriptinfos, 'Robot1 synced and waiting for robot2 ready signal')
                        waitingReadyLogPrinted = true
                    end
                end
            end
        end

    elseif MODE == 'run' then
        if not runInitialized then
            setupRun()
        end

        doRun()
    end
end


function sysCall_cleanup()
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
