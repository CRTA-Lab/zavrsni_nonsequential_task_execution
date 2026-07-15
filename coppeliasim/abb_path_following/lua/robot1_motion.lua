sim=require'sim'

-- This robot operates in FK mode

function sysCall_init()
    jh={}
    for i=1,5,1 do
        jh[i]=sim.getObject('../Joint',{index=i-1})
    end
    v=0
    vm=0.04
    a=0.01
    p=0
    startTime=20
end

function sysCall_actuation()
    local t=sim.getSimulationTime()
    if t>=startTime then
        t=t-startTime
        v=a*t
        if v>vm then v=vm end
        p=v*t
        sim.setJointPosition(jh[1],10*math.pi*math.sin(6*p)/180)
        sim.setJointPosition(jh[2],7.5*math.pi*math.sin(3*p)/180)
        sim.setJointPosition(jh[3],5*math.pi*math.sin(2.5*p)/180)
        sim.setJointPosition(jh[4],5*math.pi*math.sin(3.5*p)/180)
    end
end
