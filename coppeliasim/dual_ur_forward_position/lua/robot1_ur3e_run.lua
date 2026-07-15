MODE = 'auto' -- 'sync', 'run' ili 'auto'
AUTO_RUN_DELAY = 3 -- sekundi syncanja prije pripreme run moda
RUN_WAIT_AFTER_READY = 0.0 -- dodatno cekanje nakon robot2 ready signala

WAIT_SIGNAL = 'robot2_ready_for_robot1_run'

sim = require 'sim'
simROS2 = require('simROS2')
simIK = require 'simIK'

latestPositions = nil
sub = nil
pub = nil
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

runInitialized = false
autoSyncStartTime = nil
runAllowed = false
runStartTime = nil

function wrapToPi(a)
    while a > math.pi do
        a = a - 2 * math.pi
    end
    while a < -math.pi do
        a = a + 2 * math.pi
    end
    return a
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

function publishCurrentJoints()
    if pub then
        local pos = {}

        for i = 1, #simJoints do
            pos[i] = sim.getJointPosition(simJoints[i])
        end

        simROS2.publish(pub, {
            name = rosJointNames,
            position = pos,
            velocity = {},
            effort = {}
        })
    end
end

function setupRun()
    if runInitialized then
        return
    end

    if sub then
        simROS2.shutdownSubscription(sub)
        sub = nil
    end

    pub = simROS2.createPublisher(
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

    -- brzina kretanja od targeta do targeta
    maxLinSpeed = 0.055
    maxAngSpeed = 0.055

    -- 5 mm = 0.005 m
    arrivalThreshold = 0.005

    proxyTarget = sim.createDummy(0.01)
    sim.setObjectAlias(proxyTarget, 'proxyTarget_robot1_run')

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

    runInitialized = true
    runAllowed = false
    runStartTime = nil

    sim.addLog(sim.verbosity_scriptinfos, 'RUN READY robot1, waiting for robot2 ready signal')
end

function doRun()
    local ready = sim.getInt32Signal(WAIT_SIGNAL)

    if ready ~= 1 then
        publishCurrentJoints()
        return
    end

    if not runAllowed then
        runAllowed = true
        runStartTime = sim.getSimulationTime() + RUN_WAIT_AFTER_READY
        sim.addLog(sim.verbosity_scriptinfos, 'Robot2 ready signal received. Robot1 run starts.')
    end

    local t = sim.getSimulationTime()

    if t < runStartTime then
        publishCurrentJoints()
        return
    end

    local dt = sim.getSimulationTimeStep()

    local currentTarget = targets[currentTargetIndex]

    local pGoal = sim.getObjectPosition(currentTarget, -1)
    local pCur = sim.getObjectPosition(proxyTarget, -1)

    local dx = pGoal[1] - pCur[1]
    local dy = pGoal[2] - pCur[2]
    local dz = pGoal[3] - pCur[3]

    local dist = math.sqrt(dx * dx + dy * dy + dz * dz)

    -- kad dode do targeta, idi na sljedeci
    if dist < arrivalThreshold then
        currentTargetIndex = currentTargetIndex + 1

        if currentTargetIndex > #targets then
            currentTargetIndex = 1
        end

        currentTarget = targets[currentTargetIndex]

        pGoal = sim.getObjectPosition(currentTarget, -1)
        pCur = sim.getObjectPosition(proxyTarget, -1)

        dx = pGoal[1] - pCur[1]
        dy = pGoal[2] - pCur[2]
        dz = pGoal[3] - pCur[3]

        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        sim.addLog(sim.verbosity_scriptinfos, 'Next target: ' .. targetNames[currentTargetIndex])
    end

    -- pomicanje proxyTargeta prema trenutnom targetu
    local maxLinStep = maxLinSpeed * dt
    local pNew = {pCur[1], pCur[2], pCur[3]}

    if dist > 1e-6 then
        if dist <= maxLinStep then
            pNew = pGoal
        else
            local s = maxLinStep / dist

            pNew = {
                pCur[1] + dx * s,
                pCur[2] + dy * s,
                pCur[3] + dz * s
            }
        end
    end

    sim.setObjectPosition(proxyTarget, -1, pNew)

    -- orijentacija proxyTargeta prema trenutnom targetu
    local oCur = sim.getObjectOrientation(proxyTarget, -1)
    local oGoal = sim.getObjectOrientation(currentTarget, -1)

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

    publishCurrentJoints()
end

function sysCall_init()
    simBase = sim.getObject('/UR3e2')
    simTip = sim.getObject('/UR3e2/tool0_visual/tcp_tip_2')

    j1 = sim.getObject('/UR3e2/shoulder_pan_joint')
    j2 = sim.getObject('/UR3e2/shoulder_lift_joint')
    j3 = sim.getObject('/UR3e2/elbow_joint')
    j4 = sim.getObject('/UR3e2/wrist_1_joint')
    j5 = sim.getObject('/UR3e2/wrist_2_joint')
    j6 = sim.getObject('/UR3e2/wrist_3_joint')

    simJoints = {j1, j2, j3, j4, j5, j6}

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
        sim.addLog(sim.verbosity_scriptinfos, 'MODE=run robot1, waiting for robot2 ready signal')

    elseif MODE == 'auto' then
        sub = simROS2.createSubscription(
            '/robot1/joint_states',
            'sensor_msgs/msg/JointState',
            'jointStateCb'
        )

        sim.addLog(sim.verbosity_scriptinfos, 'MODE=auto robot1: sync first, then wait for robot2')

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
                sim.addLog(sim.verbosity_scriptinfos, 'Robot1 first joint_states received')
            end

            if sim.getSimulationTime() - autoSyncStartTime >= AUTO_RUN_DELAY then
                setupRun()
                MODE = 'run'
                sim.addLog(sim.verbosity_scriptinfos, 'AUTO switched robot1 from sync to run-waiting mode')
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

    if pub then
        simROS2.shutdownPublisher(pub)
    end

    if ikEnv then
        simIK.eraseEnvironment(ikEnv)
    end

    if proxyTarget then
        sim.removeObject(proxyTarget)
    end
end
