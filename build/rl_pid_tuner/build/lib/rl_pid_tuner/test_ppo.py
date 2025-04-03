#!/home/soham/rl_env/bin/python3
import rclpy
from stable_baselines3 import PPO
from rl_pid_tuner.rl_env import LineFollowerEnv
import os
import time

def main(args=None):
    # Initialize ROS if not already initialized
    if not rclpy.ok():
        rclpy.init(args=args)
    
    # Create environment
    env = LineFollowerEnv()
    
    # Load the trained model
    model_path = os.path.join(os.getcwd(), "logs", "best_model.zip")
    
    if not os.path.exists(model_path):
        print(f"No model found at {model_path}. Looking for alternative...")
        model_path = os.path.join(os.getcwd(), "logs", "ppo_line_follower_final.zip")
    
    if not os.path.exists(model_path):
        print(f"No model found at {model_path} either. Please check model paths.")
        return 1
    
    print(f"Loading model from {model_path}")
    model = PPO.load(model_path)
    
    # Test the model
    obs = env.reset()
    done = False
    total_reward = 0
    step_count = 0
    
    print("Testing trained model...")
    
    try:
        while not done and step_count < 1000:  # Run for 1000 steps or until done
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, done, _ = env.step(action)
            total_reward += reward
            step_count += 1
            
            # Print diagnostic information
            print(f"Step {step_count}, Reward: {reward:.4f}, Total: {total_reward:.4f}")
            print(f"Action (PID): Kp={action[0]:.4f}, Ki={action[1]:.4f}, Kd={action[2]:.4f}")
            
            # Sleep to observe behavior in real-time
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print("Test stopped by user")
    
    print(f"Test complete. Total steps: {step_count}, Total reward: {total_reward:.4f}")
    env.close()
    
    return 0

if __name__ == '__main__':
    main()