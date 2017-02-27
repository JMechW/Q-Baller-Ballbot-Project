function NForce=NoiseForce(NForceRange,Force0)
    NForce=[random('unif',-NForceRange(1),NForceRange(1));random('unif',-NForceRange(2),NForceRange(2));random('unif',-NForceRange(3),NForceRange(3))];
    NForce=[NForce,Force0(:,1:end-1)];
end