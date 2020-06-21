#!/usr/bin/env python

'''
    Training code made by Ricardo Tellez <rtellez@theconstructsim.com>
    Based on many other examples around Internet
    Visit our website at www.theconstruct.ai
'''
import gym
import time
import numpy
import random
import time
import qlearn
from gym import wrappers
import os
import cv2

# ROS packages required
import rospy
import rospkg

# import our training environment
import rl_training_env

def convert_obs_to_state_qlearn(observations):
    #input observation structure: [(blue_pos[0],blue_pos[1],blue_pos[2],blue_pos[3]),(red_pos[0],red_pos[1],red_pos[2],red_pos[3]),odom=> (x,y),speed(Twist type)]
    #this version is designed for basic qlearn. To avoid the condition that the states never appears again, classify the position of the cones into certain regions

    state = [get_approx(observations[0][0][0]), get_approx(observations[0][0][1]),get_approx(observations[0][1][0]), get_approx(observations[0][1][1]),
              get_approx(observations[0][2][0]), get_approx(observations[0][2][1]),get_approx(observations[0][3][0]),get_approx(observations[0][3][1]),
              get_approx(observations[1][0][0]), get_approx(observations[1][0][1]),get_approx(observations[1][1][0]), get_approx(observations[1][1][1]),
              get_approx(observations[1][2][0]), get_approx(observations[1][2][1]),get_approx(observations[1][3][0]),get_approx(observations[1][3][1]),
              observations[3].linear.x, observations[3].angular.z]    
    return state

def get_approx(num):
    approx = divmod(round(num),20)
    return approx[0]


if __name__ == '__main__':

    rospy.init_node('srt_rl_gym', anonymous=True, log_level=rospy.INFO)

    # Create the Gym environment
    print("start")
    env = gym.make('SrtTrainingEnv-v0')
    print("Gym environment done")
    rospy.loginfo ( "Gym environment done")

    # Set the logging system
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('rl_srt')
    outdir = pkg_path + '/training_results'
    env = wrappers.Monitor(env, outdir, force=True)
    rospy.loginfo ( "Monitor Wrapper started")

    last_time_steps = numpy.ndarray(0)

    # Loads parameters from the ROS param server
    # Parameters are stored in a yaml file inside the config directory
    # They are loaded at runtime by the launch file
    Alpha = rospy.get_param("/rl_srt/alpha")
    Epsilon = rospy.get_param("/rl_srt/epsilon")
    Gamma = rospy.get_param("/rl_srt/gamma")
    epsilon_discount = rospy.get_param("/rl_srt/epsilon_discount")
    nepisodes = rospy.get_param("/rl_srt/nepisodes")
    nsteps = rospy.get_param("/rl_srt/nsteps")

    running_step = rospy.get_param("/rl_srt/running_step")

    # Initialises the algorithm that we are going to use for learning
    qlearn = qlearn.QLearn(actions=range(env.action_space.n),
                    alpha=Alpha, gamma=Gamma, epsilon=Epsilon)
    initial_epsilon = qlearn.epsilon

    start_time = time.time()
    highest_reward = -1000
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('rl_srt')
    filename = pkg_path + "/training_results"+ "qlearn_states.npy"
    if(os.path.exists(filename)):
        print("Loading file from file:",filename)
        qlearn.load(filename)

    # Starts the main training loop: the one about the episodes to do
    for x in range(nepisodes):
        rospy.loginfo("EPISODE=>" + str(x))

        cumulated_reward = 0
        done = False
        if qlearn.epsilon > 0.05:
            qlearn.epsilon *= epsilon_discount

        # Initialize the environment and get first state of the robot
        observation = env.reset()
        converted_observation = convert_obs_to_state_qlearn(observation)
        state = ''.join(map(str, converted_observation))

        # Show on screen the actual situation of the robot
        # for each episode, we test the robot for nsteps
        for i in range(nsteps):
            #time.sleep(0.1)
            rospy.logdebug("############### Start Step=>"+str(i))
            print("############### Start Step=>"+str(i))
            # Pick an action based on the current state
            action = qlearn.chooseAction(state)
            rospy.logdebug ("Next action is:%d", action)
            # Execute the action in the environment and get feedback
            observation, reward, done, info = env.step(action)
            rospy.logdebug(str(observation) + " " + str(reward))
            cumulated_reward += reward
            print(reward)
            if highest_reward < cumulated_reward:
                highest_reward = cumulated_reward
            converted_observation = convert_obs_to_state_qlearn(observation)
            nextState = ''.join(map(str, converted_observation))

            # Make the algorithm learn based on the results
            rospy.logdebug("############### state we were=>" + str(state))
            rospy.logdebug("############### action that we took=>" + str(action))
            rospy.logdebug("############### reward that action gave=>" + str(reward))
            rospy.logdebug("############### State in which we will start next step=>" + str(nextState))
            #print("############### state we were=>" + str(state))
            #print("############### action that we took=>" + str(action))
            #print("############### reward that action gave=>" + str(reward))
            #print("############### State in which we will start next step=>" + str(nextState))
            qlearn.learn(state, action, reward, nextState)

            if not(done):
                state = nextState
            else:
                rospy.logdebug ("DONE")
                last_time_steps = numpy.append(last_time_steps, [int(i + 1)])
                break
            rospy.logdebug("############### END Step=>" + str(i))
            #raw_input("Next Step...PRESS KEY")
            #rospy.sleep(1.0)
        m, s = divmod(int(time.time() - start_time), 60)
        h, m = divmod(m, 60)
        rospy.logdebug ( ("EP: "+str(x+1)+" - [alpha: "+str(round(qlearn.alpha,2))+" - gamma: "+str(round(qlearn.gamma,2))+" - epsilon: "+str(round(qlearn.epsilon,2))+"] - Reward: "+str(cumulated_reward)+"     Time: %d:%02d:%02d" % (h, m, s)))
        qlearn.save(filename)


    rospy.loginfo ( ("\n|"+str(nepisodes)+"|"+str(qlearn.alpha)+"|"+str(qlearn.gamma)+"|"+str(initial_epsilon)+"*"+str(epsilon_discount)+"|"+str(highest_reward)+"| PICTURE |"))
    print(("\n|"+str(nepisodes)+"|"+str(qlearn.alpha)+"|"+str(qlearn.gamma)+"|"+str(initial_epsilon)+"*"+str(epsilon_discount)+"|"+str(highest_reward)+"| PICTURE |"))
    l = last_time_steps.tolist()
    l.sort()

    #print("Parameters: a="+str)
    rospy.loginfo("Overall score: {:0.2f}".format(last_time_steps.mean()))
    rospy.loginfo("Best 10 score: {:0.2f}".format(reduce(lambda x, y: x + y, l[-10:]) / len(l[-10:])))
    cv2.destroyAllWindows()
    qlearn.save(filename)
    env.close()
