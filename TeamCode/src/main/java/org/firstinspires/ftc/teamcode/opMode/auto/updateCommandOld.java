//  public void autonomousPathUpdate() {
//
//    if (!follower.isBusy() && !holding){
//        pathTimer.reset();
//        holding = true;
//        actionRun = false;
//    }
//
//        switch (pathState) {
//            case 0:
//                if (!actionRun && pathTimer.milliseconds() > SHOOT_WAIT_TIME * 5){
//                    shoot.run();
//                    actionRun = true;
//                }
//
//                if (pathTimer.milliseconds() > SHOOT_WAIT_TIME * 5 + SHOOTING_TIME) {
//                    follower.followPath(paths.BF_TL, true);
//                    intake.run();
//                    incrementPathState();
//                }
//                break;
//            case 1:
//                if(!follower.isBusy()) {
//                    endOfIntake.run();
//                    follower.followPath(paths.TL_SF, true);
//                    incrementPathState();
//                }
//                break;
//            case 2:
//                if (holding) {
//                        if (!actionRun && pathTimer.milliseconds() > SHOOT_WAIT_TIME) {
//                            shoot.run();
//                        }
//
//                        if (pathTimer.milliseconds() > SHOOT_WAIT_TIME + SHOOTING_TIME) {
//                            incrementPathState();
//                        }
//                    }
//                    break;
//            default:
//                break;
//
//        }
//    }
//    public void setPathState(int pathNum) {
//        pathState = pathNum;
//        pathTimer.reset();
//        holding = false;
//        actionRun = false;
//    }
//
//    public void incrementPathState(){
//        pathState++;
//        pathTimer.reset();
//        holding = false;
//        actionRun = false;
//
//    }