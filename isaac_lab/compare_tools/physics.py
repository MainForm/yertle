"""Same action/PhysX decimation as play; no PPO reward or auto-reset work."""

def advance(env,action):
    env.action_manager.process_action(action)
    for _ in range(env.cfg.decimation):
        env._sim_step_counter+=1
        env.action_manager.apply_action()
        env.scene.write_data_to_sim()
        env.sim.step(render=False)
        env.scene.update(dt=env.physics_dt)
    env.episode_length_buf+=1
    env.common_step_counter+=1
