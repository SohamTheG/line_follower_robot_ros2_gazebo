#!/home/soham/rl_env/bin/python3
import rclpy
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import EvalCallback
from rl_pid_tuner.rl_env import LineFollowerEnv
import os
import torch
def main(args=None):
    # Initialize ROS if not already initialized
    if not rclpy.ok():
        rclpy.init(args=args)
    
    # Create and wrap the environment
    env = LineFollowerEnv()
    log_dir = os.path.join(os.getcwd(), "logs")
    os.makedirs(log_dir, exist_ok=True)
    
    # Create a monitored environment for logging
    env = Monitor(env, log_dir)
    
    # Create evaluation environment
    eval_env = LineFollowerEnv()
    
    # Create evaluation callback
    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=log_dir,
        log_path=log_dir,
        eval_freq=1000,
        deterministic=True,
        render=False
    )

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")
    
    # Initialize PPO model with better hyperparameters for this task
    model = PPO(
        "MlpPolicy", 
        env, 
        verbose=1,
        learning_rate=0.0003,
        n_steps=100,  # Shorter steps to get more frequent updates
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        tensorboard_log=log_dir,
        device=device
    )
    
    # Train the model
    print("Starting training...")
    model.learn(total_timesteps=100000, callback=eval_callback)
    
    # Save the trained model
    model_path = os.path.join(log_dir, "ppo_line_follower_final")
    model.save(model_path)
    print(f"PPO Model training complete! Saved to {model_path}")
    
    # Close environments
    env.close()
    eval_env.close()
    
    return 0

if __name__ == '__main__':
    main()