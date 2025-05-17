function cost = costFunction(a1, a2, car1_next_state, car2_next_state, trackLength)
    collision = 0;
    if detectCollision(a1,a2)
        collision = 10^6;
    end
    d1 = trackLength - car1_next_state.pos;
    d2 = trackLength - car2_next_state.pos;
    cost = d1 - d2 + collision;
end
