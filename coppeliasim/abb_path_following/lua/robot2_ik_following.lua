sim=require'sim'
simIK=require'simIK'

-- This robot operates in IK mode

function sysCall_init()
    local simBase=sim.getObject('..')
    local simTip=sim.getObject('../tip')
    local simTarget=sim.getObject('/target2')

    ikEnv=simIK.createEnvironment()

    -- Prepare the ik group, using the convenience function 'simIK.addElementFromScene':
    ikGroup=simIK.createGroup(ikEnv)
    simIK.addElementFromScene(ikEnv,ikGroup,simBase,simTip,simTarget,simIK.constraint_position+simIK.constraint_alpha_beta)
end

function sysCall_actuation()
    simIK.handleGroup(ikEnv,ikGroup,{syncWorlds=true})
end

function sysCall_cleanup()
    simIK.eraseEnvironment(ikEnv)
end
