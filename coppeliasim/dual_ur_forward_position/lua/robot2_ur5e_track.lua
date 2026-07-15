MODE = 'auto' -- 'sync', 'track' ili 'auto'
AUTO_TRACK_DELAY = 3.0 -- sekundi syncanja prije track moda

READY_SIGNAL = 'robot2_ready_for_robot1_run'
READY_DELAY_AFTER_REACH = 0.0 -- dodatno cekanje nakon dolaska na pocetnu tocku patha

sim = require 'sim'
simIK = require 'simIK'
simROS2 = require('simROS2')

latestPositions = nil
sub = nil
pub = nil
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

pathReadySent = false
pathReachTime = nil

dx = 0
dy = 0
dz = 0

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

    -- APPROACH brzina = dolazak do pocetne tocke patha
    approachLinSpeed = 0.005
    approachAngSpeed = 0.01

    -- PATH brzina = pracenje patha nakon dolaska
    pathLinSpeed = 7
    pathAngSpeed = 7

    -- 5 mm = 0.005 m
    arrivalThreshold = 0.005

    trackState = 'approach'

    maxLinSpeed = approachLinSpeed
    maxAngSpeed = approachAngSpeed

    pub = simROS2.createPublisher(
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

    trackInitialized = true

    sim.addLog(sim.verbosity_scriptinfos, 'TRACK READY robot2')
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

    -- kada robot2 dode na pocetnu tocku patha, salje signal robot1 skripti
    if trackState == 'approach' and dist < arrivalThreshold then
        trackState = 'path'

        if pathReachTime == nil then
            pathReachTime = sim.getSimulationTime()
            sim.addLog(sim.verbosity_scriptinfos, 'ROBOT2 reached path start')
        end
    end

    if trackState == 'path' and not pathReadySent then
        if sim.getSimulationTime() - pathReachTime >= READY_DELAY_AFTER_REACH then
            sim.setInt32Signal(READY_SIGNAL, 1)
            pathReadySent = true
            sim.addLog(sim.verbosity_scriptinfos, 'ROBOT2 READY SIGNAL SENT')
        end
    end

    if trackState == 'approach' then
        maxLinSpeed = approachLinSpeed
        maxAngSpeed = approachAngSpeed
    else
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

    publishCurrentJoints()
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

        sim.addLog(sim.verbosity_scriptinfos, 'MODE=auto robot2: sync first, then track')

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
