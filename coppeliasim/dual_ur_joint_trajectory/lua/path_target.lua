sim=require'sim'
simEigen=require('simEigen')
path=require('models.path_customization-2')

function sysCall_beforeSimulation()
    path.beforeSimulation()

    dummy=sim.getObject('/UR3e2/tool0_visual/target2')
    local pathData=sim.unpackDoubleTable(sim.getBufferProperty(path.model, 'customData.PATH', {noError = true}))
    local m=simEigen.Matrix(math.floor(#pathData/7),7,pathData)
    pathPositions=m:block(1,1,m:rows(),3):data()
    pathQuaternions=m:block(1,4,m:rows(),4):data()
    pathLengths,l=sim.getPathLengths(pathPositions,3)

    v=0
    vm=0.012
    a=0.012
    p=0
    startTime=40
end

function sysCall_actuation()
    local t=sim.getSimulationTime()
    if t>=startTime then
        t=t-startTime
        v=a*t
        if v>vm then v=vm end
        p=v*t % l

        local pos=sim.getPathInterpolatedConfig(pathPositions,pathLengths,p)
        local quat=sim.getPathInterpolatedConfig(pathQuaternions,pathLengths,p,nil,{2,2,2,2})
        sim.setObjectPosition(dummy,pos,path.model)
        sim.setObjectQuaternion(dummy,quat,path.model)
    end
end
