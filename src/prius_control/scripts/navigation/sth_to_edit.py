
#################################################################################
##====================== BEGIN of POS Algorithm ===============================##
#################################################################################


               
               


                else:
                    #CHECK is collision going to happend given the currrent speed
                    pass






            #--- slow down when approaching crossroad ---#
            
            
#            if d_node < 5 and v_obs > 0.1:
                
#               decelerate()


            ### NOTE: Different drive mode??? now is energy-saving mode
            ### To avoid obstacle while try to maintain the initial(target)speed

            ### IF obstacle is coming
            if col_prob_diff(0, np.array([unit_lh_pose])) > 0: 
                print("Obstacle coming!")

                if costmap.shape[0] > 1:    # prediction mode
                    
                   # if col_prob_diff(0, np.array([unit_lh_pose])) > 0:    # obstacle approaching: is it ok to accelerate?

                        pos = update.POS()
                        if pos > 0.8:

                            accelerate()

                        ### TODO: checl there is an unavoidable collision 
                        else:
                            ### NOTE: +3 for the distance left afetr fully stopped  
                            cross_dist = get_cross_dist(np.array([unit_lh_pose])) + 3  # in meters

                            time_impact = get_impact_time(np.array([unit_lh_pose]))  # in seconds
                        
                            ### The car will collide with obs if it keeping at this car_vel
                            if cross_dist  - car_vel[0][1] * time_impact > 0:


                                ### NOTE: assume const. acceleration ! a = 1m/s (0.1m/t_res)
                                ### calculate the root of t: 1/2*a*t**2 + V_0*t - S = 0
                                roots = np.roots([1.0/2.0 * (accele), car_vel[0][1], - cross_dist])
                                ### Only one positive root since S > 0
                                root_t = np.amax(roots)

                                ### accelerate OR decelerate
                                if root_t < time_impact:
                                    
                                    accelerate()     # accelerate

                                else :
                                   
                                    decelerate()    # decelerate

                            else:
                                ### Keep moving can pass the obstacle
                                pass


                else:   ###NOTE: local planner!!! we don't have this yet

                    if prob == 1:
                        
                        decelerate()     # decelerate

            ### TODO: This is not a safe action!!!!!!!!
            elif col_prob_diff(0, np.array([unit_lh_pose])) < 0:    # obstacle leaving

                accelerate()     # accelerate

            elif prob == 1:
                
                decelerate()   # not going to move a bit
            # NO OBSTACLE AHEAD
            else:
                print("no obstacle ahead!!!!!!!!!!!!")
                
                if car_vel[0][1] + accele <= car_init_y_vel:

                    accelerate()   # accelerate

                elif car_vel[0][1] - brake >= car_init_y_vel:
                    
                    decelerate()   # decelerate



###############################################################################
##====================== END of POS Algorithm ===============================##
###############################################################################
