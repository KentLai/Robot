#include "strategy/strategy_main.h"
int main(int argc, char** argv)
{
	ros::init(argc, argv, "BBthrow");
	ros::NodeHandle nh;
	KidsizeStrategy KidsizeStrategy;
    
	ros::Rate loop_rate(30);

    Load->initparameterpath();

	while (nh.ok()) 
	{
		ros::spinOnce();
		KidsizeStrategy.strategymain();
		loop_rate.sleep();
	}
	return 0;
}

void KidsizeStrategy::strategymain()
{
	if(strategy_info->getStrategyStart())//指撥開關
	{	
		if(DIOSTRATAGAIN == false)
		{
		    BasketInfo->Init();
		    Load->LoadParameters();
		    FindballInitial();
		    DIOSTRATAGAIN = true;
		    sendbodystandflag = false;
		}		
		image();
		Draw();

		switch(BasketInfo->Robot_State)
		{
		    case Initialization:
		        ROS_INFO("Initialize");
                if(strategy_info->DIOValue.Switch.D0)
                {
                    BasketInfo->LayUpFlag = true;
                }
		        MoveContinuous(ContinuousStand);
		    	tool->Delay(1000);
                ros_com->sendBodySector(BB_StandFix);
                tool->Delay(1000);
                ros_com->sendSensorReset();
		        BasketInfo->Robot_State = Find_Ball;
		        break;
		    case Find_Ball:
		        FindballHead();
		        break;
		    case Trace_Ball:
		        TraceballHead();			
		        break;
		    case Goto_Ball:
		        TraceballBody();
		        break;
		    case Find_Target:
		        FindbasketHead();
		        break;
		    case Trace_Target:
		        TracebasketHead();
		        break;
		    case Goto_Target:
		        TracebasketBody();
		        break;
		    case UP_Basket:
				UPbasket();
				break;
			case Slum_Dunk_Basket:
				Slum_Dunk();
				break;
		    case End:
		        break;
		} 
	}
	else
	{
		if(walk_con->isStartContinuous())
        {
            walk_con->stopContinuous();
            tool->Delay(1500);
        }
        if(!BasketInfo->LoadFlag)
        {
        	BasketInfo->Init();
            Load->LoadParameters();
            BasketInfo->LoadFlag = true;
        }
        if(!sendbodystandflag)
        {
            ros_com->sendBodySector(Robot_StandUp);
            sendbodystandflag = true;
            DIOSTRATAGAIN = false;
        }
        if(strategy_info->DIOValue.Switch.D1)//2號小指撥開啟時顯示線，主要用於測試fuzzy或VisionMiddle這類的東西時使用
        {            
            image();
            Draw();
	        ROS_INFO("----------------------------------------");
            ROS_INFO("BasketInfo->Basket.size = %d", BasketInfo->Basket.size);
            ROS_INFO("BasketInfo->Basket.YMax = %d", BasketInfo->Basket.YMax);
            ROS_INFO("BasketInfo->Distancenew = %f", BasketInfo->Distancenew);
            ROS_INFO("BasketInfo->Ball.Y = %d", BasketInfo->Ball.Y);
            ROS_INFO("IMU = %f", strategy_info->getIMUValue().Yaw);
            ROS_INFO("Basket.X = %d", BasketInfo->Basket.X);
	        ROS_INFO("Ball.size = %d", BasketInfo->Ball.size);
	        ROS_INFO("----------------------------------------");
        }
        if(!BasketInfo->PrintFlag)
        {      
            std::printf("        ▃ \n");
            std::printf("　　　 　▋ 　　 ▋　 ◢▀　▀◣ \n");
            std::printf("　　　　▌　 　 　▌　▌ 　 .▌ \n");
            std::printf("　　 　 ▌　　　　▌ ▌　　　▌ \n");
            std::printf("　 　　▐ 　 　　 ▌ ▌ 　 　▌ \n");
            std::printf("　 　　 ▐ 　 　 ▀■▀ 　 　▌ \n");
            std::printf("　　　◢◤　　　　　　　　　▀▃ \n");
            std::printf("　　◢◤　　　　　　　　　 　　◥◣ \n");
            std::printf("　　▌　　　　　　　　　　 　　　▌ \n");
            std::printf("　 ▐　 　●　　 　　　　●　　　　▌ 　 \n");
            std::printf("　　▌　　　　　　　　　　　　　 ▌ \n");
            std::printf("　　◥◣ 　 　　 ╳ 　　　　　　◢◤ \n");
            std::printf("　　 ◢▀▅▃▂　　　  ▂▂▃▅▀▅ ◢◤\n");
            std::printf("　◢◤　　　　▀▀▀▀▀　　　　　◥◣ \n");
            std::printf("▐◣▃▌　　　　　　　　　　　▐▃◢▌ \n");
            std::printf("◥◣▃▌　　　　 　　 　　　　▐▃◢◤ \n");
            std::printf("　 ▀▅▃　　　　　 　 　　▂▅▀ \n");
            std::printf("　　 　 ▀■▆▅▅▅▅▅▆■█▀ \n");
            std::printf("　　　 　▐▃▃▃▲▃▃▃◢ \n\n\n");
            std::printf("-----------------------------------\n");
            std::printf("        ▐ ▀◣     ◢▀ ◣\n");
            std::printf("       ◢　  ▌    ▌　  ▌\n");
            std::printf("        ▌　  ▌   ▀ 　  ▌\n");
            std::printf("         ▀     ▃ ▌ 　　 ▌▅▃▂▂▂▂▂\n");
            std::printf("          ▌                     ▀▌\n");
            std::printf("  　   　 ◢◤　　　　　　　　　    ▌\n");
            std::printf("  　   　◤　　　　　　   ● 　　　 ▐ \n");
            std::printf("        ▌　 　  ●　              ▂▐ \n");
            std::printf("       ▌　 　  　　 　　　　   ▂▂▌\n");
            std::printf("        ▌　 　  　　 W　　　  ▌ 　   \n");
            std::printf("  　   　◥◣ 　 　　    　　 ◢◤        \n");
            std::printf("    　    ◢▀▅▃▂　　　  ▂ ◢◤       ◢◤\n");
            std::printf(" ◥◣    ◢◤　　　　▀▀▀▀▀　  ◥◣▂   ◢◤\n");
            std::printf("   ◥◣ ▐◤                    ▐ ◢◤\n");
            std::printf("      ▌                      ▐\n");
            std::printf("      ▌　　　 　　 　　　　  ▐\n");
            std::printf("      ▐                      ▐\n"); 
            std::printf("        ▌  　　　　 　　 　　▌\n");
            std::printf("  　     ▀▀▅▃　　　　　 　▂▅▀\n");
            std::printf("  　　    　 ▀■▆▅▅▅▅▅▆■█▀ \n");
            std::printf("  	        ▌        ▌\n");  
            std::printf("                ██       ██ \n");
            BasketInfo->PrintFlag = true;
        }
	}
}

void KidsizeStrategy::Draw()//顯示出籃框的線&&球模的線
{
    ros_com->drawImageFunction(1, DrawMode::DrawLine, 0, 320, 120, 120, 152, 245, 255);
    ros_com->drawImageFunction(2, DrawMode::DrawLine, BasketInfo->BasketVerticalBaseLine50, BasketInfo->BasketVerticalBaseLine50, 0, 240, 255, 0, 0);//red
    ros_com->drawImageFunction(3, DrawMode::DrawLine, BasketInfo->BasketVerticalBaseLine60, BasketInfo->BasketVerticalBaseLine60, 0, 240, 255, 0, 0);//50
    ros_com->drawImageFunction(4, DrawMode::DrawLine, BasketInfo->BasketVerticalBaseLine70, BasketInfo->BasketVerticalBaseLine70, 0, 240, 255, 0, 0);//60
    ros_com->drawImageFunction(5, DrawMode::DrawLine, BasketInfo->BasketVerticalBaseLine80, BasketInfo->BasketVerticalBaseLine80, 0, 240, 255, 0, 0);//70
    ros_com->drawImageFunction(6, DrawMode::DrawLine, BasketInfo->BasketVerticalBaseLine , BasketInfo->BasketVerticalBaseLine , 0, 240, 255, 0, 0);//80
    ros_com->drawImageFunction(7, DrawMode::DrawObject, BasketInfo->Ball.XMin, BasketInfo->Ball.XMax, BasketInfo->Ball.YMin, BasketInfo->Ball.YMax, 255, 0, 255);//pink
    ros_com->drawImageFunction(8, DrawMode::DrawObject, BasketInfo->Basket.XMin, BasketInfo->Basket.XMax, BasketInfo->Basket.YMin, BasketInfo->Basket.YMax, 255, 255, 0);//yellow
    ros_com->drawImageFunction(9, DrawMode::DrawLine, BasketInfo->Basket.X, BasketInfo->Basket.X, 0, 240, 255, 255, 0);//80
}

void KidsizeStrategy::MoveHead(HeadMotorID ID, int Position, int Speed)//動頭(馬達編號，刻度，速度)
{
    ros_com->sendHeadMotor(ID,Position,Speed);
    tool->Delay(50);
    if(ID == HeadMotorID::HorizontalID)
    {
        BasketInfo->HorizontalHeadPosition = Position;
    }
    else
    {
        BasketInfo->VerticalHeadPosition = Position;
    }
}

void KidsizeStrategy::MoveBody(int Mode, int TurnValue)//單步步態，根據ini檔裡面的參數來調整x,y,z,theda,IMU
{
    if(Mode == TurnLeft)
    {
        ros_com->sendBodyAuto(BasketInfo->Walking[Mode].x + (TurnValue-1)*200, BasketInfo->Walking[Mode].y, BasketInfo->Walking[Mode].z, BasketInfo->TurnValue[TurnLeft*TurnMode + TurnValue], WalkingMode(BasketInfo->Walking[Mode].mode), SensorMode(BasketInfo->Walking[Mode].IMUSet));
    }
    else if(Mode == TurnRight)
    {
        ros_com->sendBodyAuto(BasketInfo->Walking[Mode].x, BasketInfo->Walking[Mode].y, BasketInfo->Walking[Mode].z, BasketInfo->TurnValue[TurnRight*TurnMode + TurnValue], WalkingMode(BasketInfo->Walking[Mode].mode), SensorMode(BasketInfo->Walking[Mode].IMUSet));
    }
    else
    {
        ros_com->sendBodyAuto(BasketInfo->Walking[Mode].x, BasketInfo->Walking[Mode].y, BasketInfo->Walking[Mode].z, BasketInfo->Walking[Mode].theta, WalkingMode(BasketInfo->Walking[Mode].mode), SensorMode(BasketInfo->Walking[Mode].IMUSet));
    }
}

void KidsizeStrategy::MoveContinuous(int mode)//連續步態，根據ini檔裡面的參數來調整x,y,z,theda,IMU
{
    if(BasketInfo->Robot_State==Initialization)
    {
        walk_con->setWalkParameterInit(BasketInfo->ContinuousStep[mode].ContinuousInit.InitX, BasketInfo->ContinuousStep[mode].ContinuousInit.InitY, BasketInfo->ContinuousStep[mode].ContinuousInit.InitZ, \
                                       BasketInfo->ContinuousStep[mode].ContinuousInit.InitTheta);//連續步態初始值
        walk_con->setWalkParameterMax(3000, 2000, 0, 16);//連續步態上限
        walk_con->setWalkParameterMin(-3000, -2000, 0, -16);//連續步態下限
        walk_con->setWalkParameterExp(BasketInfo->ContinuousStep[mode].ContinuousMove.ExpX, BasketInfo->ContinuousStep[mode].ContinuousMove.ExpY, BasketInfo->ContinuousStep[mode].ContinuousMove.ExpZ, BasketInfo->ContinuousStep[mode].ContinuousMove.ExpTheta);//連續步態期望值
        walk_con->setWalkParameterOneAddValueAndPeriod(BasketInfo->ContinuousStep[mode].ContinuousMove.AddX, BasketInfo->ContinuousStep[mode].ContinuousMove.AddY, BasketInfo->ContinuousStep[mode].ContinuousMove.AddZ, BasketInfo->ContinuousStep[mode].ContinuousMove.AddTheta, \
                                                         BasketInfo->AddPeriod);//連續步態x,y,z,theda，每?秒+多少期望值
    }
    else if(BasketInfo->ContinuousStep[mode].ContinuousMove.ChangeMode) 
    {
        walk_con->setWalkParameterExp(BasketInfo->ContinuousStep[mode].ContinuousMove.ExpX, BasketInfo->ContinuousStep[mode].ContinuousMove.ExpY, BasketInfo->ContinuousStep[mode].ContinuousMove.ExpZ, BasketInfo->ContinuousStep[mode].ContinuousMove.ExpTheta);//連續步態期望值
        walk_con->setWalkParameterOneAddValueAndPeriod(BasketInfo->ContinuousStep[mode].ContinuousMove.AddX, BasketInfo->ContinuousStep[mode].ContinuousMove.AddY, BasketInfo->ContinuousStep[mode].ContinuousMove.AddZ, BasketInfo->ContinuousStep[mode].ContinuousMove.AddTheta, \
                                                         BasketInfo->AddPeriod);//連續步態x,y,z,theda，每?秒+多少期望值
    }
    else
    {
        walk_con->setWalkParameterExp(BasketInfo->ContinuousStep[ContinuousStay].ContinuousMove.ExpX+BasketInfo->ContinuousStep[mode].ContinuousMove.ExpX, BasketInfo->ContinuousStep[ContinuousStay].ContinuousMove.ExpY+BasketInfo->ContinuousStep[mode].ContinuousMove.ExpY, BasketInfo->ContinuousStep[ContinuousStay].ContinuousMove.ExpZ+BasketInfo->ContinuousStep[mode].ContinuousMove.ExpZ, BasketInfo->ContinuousStep[ContinuousStay].ContinuousMove.ExpTheta+BasketInfo->ContinuousStep[mode].ContinuousMove.ExpTheta);//連續步態期望值
        walk_con->setWalkParameterOneAddValueAndPeriod(BasketInfo->ContinuousStep[mode].ContinuousMove.AddX, BasketInfo->ContinuousStep[mode].ContinuousMove.AddY, BasketInfo->ContinuousStep[mode].ContinuousMove.AddZ, BasketInfo->ContinuousStep[mode].ContinuousMove.AddTheta, \
                                                         BasketInfo->AddPeriod);//連續步態x,y,z,theda，每?秒+多少期望值
    }
}

void KidsizeStrategy::MeasureDistance()//測距
{
    if (BasketInfo->Basket.size > Basketfarsize)
    {
        BasketInfo->MiddleAngle = atan(((double)BasketInfo->VisionMiddle/(double)BasketInfo->RobotSearchBasketHeight));
        BasketInfo->BAngle = atan((double)BasketInfo->ScreenButtom/(double)BasketInfo->RobotSearchBasketHeight);
        BasketInfo->AAngel = ( BasketInfo->MiddleAngle - BasketInfo->BAngle );
        BasketInfo->MiddleAngle = BasketInfo->MiddleAngle + BasketInfo->FeedBackError;

        if(BasketInfo->Basket.YMax > 120)
        {
            BasketInfo->VisionBasketDistance = BasketInfo->Basket.YMax - 120;
            BasketInfo->dyAngle = atan2((double)((BasketInfo->VisionBasketDistance)*(tan((double)BasketInfo->AAngel))),120);
            BasketInfo->BasketAngle = ( BasketInfo->MiddleAngle - BasketInfo->dyAngle );
        }
        else
        {	
            BasketInfo->VisionBasketDistance = 120 - BasketInfo->Basket.YMax;
            BasketInfo->dyAngle = atan2((double)((BasketInfo->VisionBasketDistance)*(tan((double)BasketInfo->AAngel))),120);
            BasketInfo->BasketAngle = ( BasketInfo->MiddleAngle + BasketInfo->dyAngle );
        }

        BasketInfo->Distancenew = abs((double)BasketInfo->RobotSearchBasketHeight*tan((double)BasketInfo->BasketAngle));
    }
    else
    {
        BasketInfo->Distancenew = 80;
    }
    
    //return BasketInfo->Basketdistance;
}

void KidsizeStrategy::HeadMeasureDistance()
{
    MoveHead(HeadMotorID::HorizontalID,2048, 200);
    image();
    while(abs(BasketInfo->Basket.Y - 120) > 0)//abs(BasketInfo->Basket.Y - 120) > 1
    {
        if(BasketInfo->Basket.Y == 0)
        {
            MoveHead(HeadMotorID::VerticalID,1800, 200);
        } 
        else if((BasketInfo->Basket.Y - 120) > 0)// else if((BasketInfo->Basket.Y - 120) > 1)
        {
            MoveHead(HeadMotorID::VerticalID,BasketInfo->VerticalHeadPosition - 1, 200);
        }
        else if((BasketInfo->Basket.Y - 120) < 0)
        {
            MoveHead(HeadMotorID::VerticalID,BasketInfo->VerticalHeadPosition + 1, 200);
        }
        image();
        ROS_INFO("Basket Y = %d", BasketInfo->Basket.Y);
    }
    BasketInfo->HeadVerticalAngle = (double)(BasketInfo->VerticalHeadPosition - 1024) * Scale2Deg + BasketInfo->RobotStandFeedBack + BasketInfo->FeedBackError;
    ROS_INFO("VerticalHeadPosition = %d", BasketInfo->VerticalHeadPosition);
    ROS_INFO("HeadVerticalAngle = %lf", BasketInfo->HeadVerticalAngle);
    ROS_INFO("DistanceError = %lf", BasketInfo->DistanceError);
    BasketInfo->Distancenew = (BasketInfo->RobotHeight + CameraHeight * sin(BasketInfo->HeadVerticalAngle * Rad)) * tan(BasketInfo->HeadVerticalAngle * Rad) + CameraHeight * cos(BasketInfo->HeadVerticalAngle * Rad) + BasketInfo->DistanceError;
}

void KidsizeStrategy::image()//影像辨識，用於辨識球模or籃框模
{
    strategy_info->get_image_flag = true;
	ros::spinOnce();
    //初始值都先給0
    BasketInfo->Ball.size = 0;
    BasketInfo->Ball.XMin = 0;
    BasketInfo->Ball.XMax = 0;
    BasketInfo->Ball.YMin = 0;
    BasketInfo->Ball.YMax = 0;
    BasketInfo->Ball.X    = 0;
    BasketInfo->Ball.Y    = 0;
    BasketInfo->Basket.size  = 0;
    BasketInfo->Basket.size = 0;
    BasketInfo->Basket.XMin = 0;
    BasketInfo->Basket.XMax = 0;
    BasketInfo->Basket.YMin = 0;
    BasketInfo->Basket.YMax = 0;
    BasketInfo->Basket.X    = 0;
    BasketInfo->Basket.Y    = 0;

    for( int i = 0 ; i < strategy_info->color_mask_subject_cnts[BasketInfo->Ballcolor] ;i++)//球模
    {
        if (strategy_info->color_mask_subject[BasketInfo->Ballcolor][i].size > Ballfarsize && strategy_info->color_mask_subject[BasketInfo->Ballcolor][i].size < 40000 )//球模
        {
            BasketInfo->Ball.XMin = strategy_info->color_mask_subject[BasketInfo->Ballcolor][i].XMin ;
            BasketInfo->Ball.XMax = strategy_info->color_mask_subject[BasketInfo->Ballcolor][i].XMax ;
            BasketInfo->Ball.YMin = strategy_info->color_mask_subject[BasketInfo->Ballcolor][i].YMin ;
            BasketInfo->Ball.YMax = strategy_info->color_mask_subject[BasketInfo->Ballcolor][i].YMax ;
            BasketInfo->Ball.X    = strategy_info->color_mask_subject[BasketInfo->Ballcolor][i].X ;
            BasketInfo->Ball.Y    = strategy_info->color_mask_subject[BasketInfo->Ballcolor][i].Y ;
            BasketInfo->Ball.size = strategy_info->color_mask_subject[BasketInfo->Ballcolor][i].size;
        }
    }

    for (int i = 0; i < strategy_info->color_mask_subject_cnts[BasketInfo->Basketcolor];i++)//籃框模
    {
        // ROS_INFO("BasketInfo->Basket.size = %d",strategy_info->color_mask_subject[BasketInfo->Basketcolor][i].size);
        if (strategy_info->color_mask_subject[BasketInfo->Basketcolor][i].size > Basketfarsize)//籃框模  && ((strategy_info->color_mask_subject[BasketInfo->Basketcolor][i].YMax-strategy_info->color_mask_subject[BasketInfo->Basketcolor][i].YMin)/(strategy_info->color_mask_subject[BasketInfo->Basketcolor][i].XMax-strategy_info->color_mask_subject[BasketInfo->Basketcolor][i].XMin))>0.5 &&((strategy_info->color_mask_subject[BasketInfo->Basketcolor][i].YMax-strategy_info->color_mask_subject[BasketInfo->Basketcolor][i].YMin)/(strategy_info->color_mask_subject[BasketInfo->Basketcolor][i].XMax-strategy_info->color_mask_subject[BasketInfo->Basketcolor][i].XMin))<1.5
        {
            BasketInfo->Basket.XMin = strategy_info->color_mask_subject[BasketInfo->Basketcolor][i].XMin ;
            BasketInfo->Basket.XMax = strategy_info->color_mask_subject[BasketInfo->Basketcolor][i].XMax ;
            BasketInfo->Basket.YMin = strategy_info->color_mask_subject[BasketInfo->Basketcolor][i].YMin;
            BasketInfo->Basket.YMax = strategy_info->color_mask_subject[BasketInfo->Basketcolor][i].YMax;
            BasketInfo->Basket.X    = strategy_info->color_mask_subject[BasketInfo->Basketcolor][i].X ;
            BasketInfo->Basket.Y    = strategy_info->color_mask_subject[BasketInfo->Basketcolor][i].Y ;
            BasketInfo->Basket.size  = strategy_info->color_mask_subject[BasketInfo->Basketcolor][i].size;
            BasketInfo->Basketsize = (BasketInfo->Basket.XMax - BasketInfo->Basket.XMin)*(BasketInfo->Basket.YMax-BasketInfo->Basket.YMin);
        }
    }
}

float KidsizeStrategy::CompensateDistant()//輔助測距，以Basket.size來判別現在位置在哪個區間，以此完成測距
{
    // ROS_INFO("---\tstart compensating\t---");
    double front = 0, back = 0, EstimatedDistance = 0;
    ROS_INFO("BasketInfo->Basket.size = %d",BasketInfo->Basket.size);
    if(BasketInfo->Basket.size > BasketInfo->SizeOfDist[0])
    {
        ROS_INFO("40-50");
        front = 40;
        back = 50;
    }
    else if(BasketInfo->Basket.size > BasketInfo->SizeOfDist[1])
    {
        ROS_INFO("50-60");
        front = 50;
        back = 60;
    }
    else if(BasketInfo->Basket.size > BasketInfo->SizeOfDist[2])
    {
        ROS_INFO("60-70");
        front = 60;
        back = 70;
    }
    else if(BasketInfo->Basket.size > BasketInfo->SizeOfDist[3])
    {
        ROS_INFO("70-80");
        front = 70;
        back = 80;
    }
    else
    {
        ROS_INFO("80-");
        front = 80;
        back = 90;
    }
    EstimatedDistance = back - (10*(double)BasketInfo->Basket.size - BasketInfo->SizeOfDist[(int)((back-50)/10)]\
                            /(double)BasketInfo->SizeOfDist[(int)((front-50)/10)]-BasketInfo->SizeOfDist[(int)((back-50)/10)]);
    std::printf("\033[0;33mestimate by size : %f\033[0m\n",EstimatedDistance);

/* 
    if(abs(EstimatedDistance - BasketInfo->Distancenew) > 5)
    {
        BasketInfo->Distancenew = EstimatedDistance;
        ROS_INFO("differ between estimated distant and computed distant is large than 5, set distant to %f",EstimatedDistance);
        BasketInfo-> ReaimFlag = true;
    }
    else if(back == 90)
    {
        BasketInfo->Distancenew = 80;
        ROS_INFO("distant > 80, set distant to 80");
        BasketInfo-> ReaimFlag = true;
    }
    else if(BasketInfo->Distancenew >= front && BasketInfo->Distancenew < back)
    {
        MoveContinuous(ContinuousForward);
        BasketInfo->Distancenew = BasketInfo->Distancenew;
        ROS_INFO("Good distant");
    }*/
    

        BasketInfo->Distancenew = EstimatedDistance;
        ROS_INFO("estimated distant = %f",BasketInfo->Distancenew);
        BasketInfo-> ReaimFlag = true;
    return BasketInfo->Distancenew;
    // ROS_INFO("---\tend\t---");
}

void KidsizeStrategy::ComputeFuzzy()//計算力道，利用權重算法，ex:距離52，50的力道*0.8+60的力道*0.2(類似這種概念)，
{
    ROS_INFO("---\tstart computing\t---");
    BasketInfo->disint = BasketInfo->Distancenew;
    if (BasketInfo->disint <= BasketInfo->dis35_x)								
    {
        ROS_INFO("dist <= 35");
        BasketInfo->weight_35 = 1.0;
        BasketInfo->weight_40 = 0.0;
        BasketInfo->weight_50 = 0.0;
        BasketInfo->weight_60 = 0.0;
        BasketInfo->weight_61 = 0.0;
        BasketInfo->weight_70 = 0.0;
        BasketInfo->weight_71 = 0.0;
        BasketInfo->weight_80 = 0.0;
        BasketInfo->weight_81 = 0.0;
        BasketInfo->weight_90 = 0.0;
    }
    else if ((BasketInfo->disint >= BasketInfo->dis35_x) && (BasketInfo->disint < BasketInfo->dis40_x))		
    {
        ROS_INFO("35 <= dist < 40");
        BasketInfo->weight_35 = (BasketInfo->dis40_x - BasketInfo->disint) / (BasketInfo->dis40_x - BasketInfo->dis35_x);
        BasketInfo->weight_40 = (BasketInfo->disint - BasketInfo->dis35_x) / (BasketInfo->dis40_x - BasketInfo->dis35_x);
        BasketInfo->weight_50 = 0.0;
        BasketInfo->weight_60 = 0.0;
        BasketInfo->weight_70 = 0.0;
        BasketInfo->weight_71 = 0.0;
        BasketInfo->weight_80 = 0.0;
        BasketInfo->weight_81 = 0.0;
        BasketInfo->weight_90 = 0.0;
    }
    else if ((BasketInfo->disint >= BasketInfo->dis40_x) && (BasketInfo->disint < BasketInfo->dis50_x))		
    {
        ROS_INFO("40 <= dist < 50");
        BasketInfo->weight_35 = 0.0;
        BasketInfo->weight_40 = (BasketInfo->dis50_x - BasketInfo->disint) / (BasketInfo->dis50_x - BasketInfo->dis40_x);
        BasketInfo->weight_50 = (BasketInfo->disint - BasketInfo->dis40_x) / (BasketInfo->dis50_x - BasketInfo->dis40_x);
        BasketInfo->weight_60 = 0.0;
        BasketInfo->weight_61 = 0.0;
        BasketInfo->weight_70 = 0.0;
        BasketInfo->weight_71 = 0.0;
        BasketInfo->weight_80 = 0.0;
        BasketInfo->weight_81 = 0.0;
        BasketInfo->weight_90 = 0.0;
    }
    else if ((BasketInfo->disint >= BasketInfo->dis50_x) && (BasketInfo->disint < BasketInfo->dis60_x))		
    {
        ROS_INFO("50 <= dist < 60");
        BasketInfo->weight_35 = 0.0;
        BasketInfo->weight_40 = 0.0;
        BasketInfo->weight_50 = (BasketInfo->dis60_x - BasketInfo->disint) / (BasketInfo->dis60_x - BasketInfo->dis50_x);
        BasketInfo->weight_60 = (BasketInfo->disint - BasketInfo->dis50_x) / (BasketInfo->dis60_x - BasketInfo->dis50_x);
        BasketInfo->weight_61 = 0.0;
        BasketInfo->weight_70 = 0.0;
        BasketInfo->weight_71 = 0.0;
        BasketInfo->weight_80 = 0.0;
        BasketInfo->weight_81 = 0.0;
        BasketInfo->weight_90 = 0.0;
    }
    else if ((BasketInfo->disint >= BasketInfo->dis60_x) && (BasketInfo->disint < BasketInfo->dis70_x))		
    {
        ROS_INFO("60 <= dist < 70");
        BasketInfo->weight_35 = 0.0;
        BasketInfo->weight_40 = 0.0;
        BasketInfo->weight_50 = 0.0;
        BasketInfo->weight_60 = 0.0;
        BasketInfo->weight_61 = (BasketInfo->dis70_x -BasketInfo-> disint) / (BasketInfo->dis70_x - BasketInfo->dis60_x);
        BasketInfo->weight_70 = (BasketInfo->disint - BasketInfo->dis60_x) / (BasketInfo->dis70_x - BasketInfo->dis60_x);
        BasketInfo->weight_71 = 0.0;
        BasketInfo->weight_80 = 0.0;
        BasketInfo->weight_81 = 0.0;
        BasketInfo->weight_90 = 0.0;
    }
    else if ((BasketInfo->disint >= BasketInfo->dis70_x) && (BasketInfo->disint < BasketInfo->dis80_x))		
    {
        ROS_INFO("70 <= dist < 80");
        BasketInfo->weight_35 = 0.0;
        BasketInfo->weight_40 = 0.0;
        BasketInfo->weight_50 = 0.0;
        BasketInfo->weight_60 = 0.0;
        BasketInfo->weight_61 = 0.0;
        BasketInfo->weight_70 = 0.0;
        BasketInfo->weight_71 = (BasketInfo->dis80_x - BasketInfo->disint) / (BasketInfo->dis80_x - BasketInfo->dis70_x);
        BasketInfo->weight_80 = (BasketInfo->disint - BasketInfo->dis70_x) / (BasketInfo->dis80_x - BasketInfo->dis70_x);
        BasketInfo->weight_81 = 0.0;
        BasketInfo->weight_90 = 0.0;
    }
    else if ((BasketInfo->disint >= BasketInfo->dis80_x) && (BasketInfo->disint < BasketInfo->dis90_x))		
    {
        ROS_INFO("80 <= dist < 90");
        BasketInfo->weight_35 = 0.0;
        BasketInfo->weight_40 = 0.0;
        BasketInfo->weight_50 = 0.0;
        BasketInfo->weight_60 = 0.0;
        BasketInfo->weight_61 = 0.0;
        BasketInfo->weight_70 = 0.0;
        BasketInfo->weight_71 = 0.0;
        BasketInfo->weight_80 = 0.0;
        BasketInfo->weight_81 = (BasketInfo->dis90_x - BasketInfo->disint) / (BasketInfo->dis90_x - BasketInfo->dis80_x);
        BasketInfo->weight_90 = (BasketInfo->disint - BasketInfo->dis80_x) / (BasketInfo->dis90_x - BasketInfo->dis80_x);
    }
    else if (BasketInfo->disint >= BasketInfo->dis90_x)						
    {
        ROS_INFO("dist >= 90");
        BasketInfo->weight_35 = 0.0;
        BasketInfo->weight_40 = 0.0;
        BasketInfo->weight_50 = 0.0;
        BasketInfo->weight_60 = 0.0;
        BasketInfo->weight_61 = 0.0;
        BasketInfo->weight_70 = 0.0;
        BasketInfo->weight_71 = 0.0;
        BasketInfo->weight_80 = 0.0;
        BasketInfo->weight_81 = 0.0;
        BasketInfo->weight_90 = 1.0;
    }

    BasketInfo->disspeed =  BasketInfo->weight_35*BasketInfo->dis35speed + BasketInfo->weight_40*BasketInfo->dis40speed + \
                            BasketInfo->weight_50*BasketInfo->dis50speed + BasketInfo->weight_60*BasketInfo->dis60speed + \
                            BasketInfo->weight_61*BasketInfo->dis61speed + BasketInfo->weight_70*BasketInfo->dis70speed + \
                            BasketInfo->weight_71*BasketInfo->dis71speed + BasketInfo->weight_80*BasketInfo->dis80speed + \
                            BasketInfo->weight_81*BasketInfo->dis81speed + BasketInfo->weight_90*BasketInfo->dis90speed;
    ROS_INFO("---\tfinish computing, the speed is %d\t---",BasketInfo->disspeed);
}

void KidsizeStrategy::SelectBaseLine()//以測距後的值來對BasketVerticalBaseLine
{
    if(BasketInfo->Distancenew > 80)
	{
		BasketInfo->BasketVerticalBaseLine = BasketInfo->BasketVerticalBaseLine80;
	}
	else if(BasketInfo->Distancenew > 70)
	{
		BasketInfo->BasketVerticalBaseLine = BasketInfo->BasketVerticalBaseLine70;
	}
	else if(BasketInfo->Distancenew > 60)
	{
		BasketInfo->BasketVerticalBaseLine = BasketInfo->BasketVerticalBaseLine60;
	}
	else if(BasketInfo->Distancenew > 50)
	{
		BasketInfo->BasketVerticalBaseLine = BasketInfo->BasketVerticalBaseLine50;
	}
    else
    {
        BasketInfo->BasketVerticalBaseLine = BasketInfo->BasketVerticalBaseLine;
    }

    if(BasketInfo->Turn > 0)//這邊是怕說旋轉會導致BaseLine的不同以此而做的調整，視情況調整
    {
        BasketInfo->BasketVerticalBaseLine += BasketInfo->Turn * 0;
    }
    else if(BasketInfo->Turn < 0)
    {
        BasketInfo->BasketVerticalBaseLine -= BasketInfo->Turn * 0;
    }

    if(BasketInfo->RobotPosition == BigGOAhead)//看球的初始方向在哪，以此來考慮是否要幫baseline加補償值
    {
        BasketInfo->BasketVerticalBaseLine = BasketInfo->BasketVerticalBaseLine;
    }
    else if(BasketInfo->RobotPosition == TurnLeft)//同上
    {
        BasketInfo->BasketVerticalBaseLine += 0;
    }
    else if(BasketInfo->RobotPosition == TurnRight)//同上
    {
        BasketInfo->BasketVerticalBaseLine += 0;
    }

}

void KidsizeStrategy::FindballInitial()//找球時，頭的初始位置
{
    MoveHead(HeadMotorID::VerticalID, 1623, 200);
    MoveHead(HeadMotorID::HorizontalID, 2651, 200);
}

void KidsizeStrategy::FindballHead()
{
    if(BasketInfo->Ball.size > Ballfarsize)//找到球了，開始追蹤球
    {
        ROS_INFO("FIND BALL");
        BasketInfo->Robot_State = Trace_Ball;
    }
    else
    {
        switch (BasketInfo->HeadState)//頭垂直狀態，抬頭&&低頭
        {
        case etNear:       
            MoveHead(HeadMotorID::VerticalID, 1575, 200);       
            break;
        case etClose:     
            MoveHead(HeadMotorID::VerticalID, 1100, 200);       
            break;
        }
        switch(BasketInfo->HeadTurnSide)
        {
        case HaedTurnRight://頭水平狀態，以右轉為例，轉到頭的刻度-HeadTurnSpeed<=HorizontalMinAngle時，水平狀態&&垂直狀態改變，以此循環，直到Ball.size > Ballfarsize
            if((BasketInfo->HorizontalHeadPosition - BasketInfo->HeadTurnSpeed) > BasketInfo->HorizontalMinAngle)
            {          
                MoveHead(HeadMotorID::HorizontalID, BasketInfo->HorizontalHeadPosition - BasketInfo->HeadTurnSpeed, 200);
            }
            else if((BasketInfo->HorizontalHeadPosition - BasketInfo->HeadTurnSpeed) <= BasketInfo->HorizontalMinAngle)
            {          
                BasketInfo->HeadTurnSide = HeadTurnLeft;
                BasketInfo->HeadState = (BasketInfo->HeadState+1)%2;
            }
            break;

        case HeadTurnLeft:
            if((BasketInfo->HorizontalHeadPosition + BasketInfo->HeadTurnSpeed) < BasketInfo->HorizontalMaxAngle)
            {           
                MoveHead(HeadMotorID::HorizontalID, BasketInfo->HorizontalHeadPosition + BasketInfo->HeadTurnSpeed, 200);        
            }
            else if((BasketInfo->HorizontalHeadPosition + BasketInfo->HeadTurnSpeed) >= BasketInfo->HorizontalMaxAngle)
            {       
                BasketInfo->HeadTurnSide = HaedTurnRight;
                BasketInfo->HeadState = (BasketInfo->HeadState+1)%2;
            }
            break;
        }
    }   
}

void KidsizeStrategy::TraceballHead()//頭追蹤球
{
    if(BasketInfo->Ball.size <= Ballfarsize)//沒找到球時切回Find_Ball
    {
        ROS_INFO("Miss Ball");
        walk_con->stopContinuous();
        tool->Delay(1500);
        BasketInfo->StraightCatchFlag = true;
        BasketInfo->PreRotateFlag = false;
        BasketInfo->Robot_State = Find_Ball;
    }
    else
    {
        BasketInfo->BallMoveX = BasketInfo->Ball.X - BasketInfo->BallVerticalBaseLine;//可以當作與球baseline的差
        BasketInfo->BallMoveY = BasketInfo->Ball.Y - BasketInfo->BallHorizontalBaseLine;
        BasketInfo->ErrorHorizontalAngle = BasketInfo->ImgHorizontalAngle * (double)BasketInfo->BallMoveX / (double)RobotVisionWidth;//馬達轉攝影機320pixel時轉的角度*與球baseline的差/320pixel,算出會得到角度
        BasketInfo->ErrorVerticalAngle = BasketInfo->ImgVerticalAngle * (double)BasketInfo->BallMoveY / (double)RobotVisionHeight;//馬達轉攝影機240pixel時轉的角度*與球baseline的差/240pixel,算出會得到角度
        MoveHead(HeadMotorID::HorizontalID, BasketInfo->HorizontalHeadPosition - (MotorMoveOneDegree * BasketInfo->ErrorHorizontalAngle *TraceDegreePercent*1), 200);//再利用上面得到的角度來換算成刻度，來call   MoveHead()
        MoveHead(HeadMotorID::VerticalID, BasketInfo->VerticalHeadPosition - (MotorMoveOneDegree * BasketInfo->ErrorVerticalAngle * TraceDegreePercent*1), 200);
        if(abs(BasketInfo->Ball.X - BasketInfo->BallVerticalBaseLine) <= BasketInfo->BallVerticalError && abs(BasketInfo->Ball.Y - BasketInfo->BallHorizontalBaseLine) <= BasketInfo->BallHorizontalError)//當誤差小於Error時狀態切到Goto_Ball
        {
            if(BasketInfo->StraightCatchFlag)
            {
                if(BasketInfo->HorizontalHeadPosition > (2048 - 408) && BasketInfo->HorizontalHeadPosition < (2048 + 500) && BasketInfo->VerticalHeadPosition <= BasketInfo->CatchBallLine  && !walk_con->isStartContinuous())
                {
                    std::printf("\033[0;33mCatch Ball\033[0m\n");
                    BasketInfo->PreRotateFlag = true; 
                    BasketInfo->ContinuousFlag = false;
                    BasketInfo->StoopFlag = true;
                    BasketInfo->Robot_State = Goto_Ball;
                }
                BasketInfo->StraightCatchFlag = false;   
            }
            if(!BasketInfo->PreRotateFlag)
            {
                if(!walk_con->isStartContinuous())
                {
                    walk_con->startContinuous((WalkingMode)BasketInfo->ContinuousStep[ContinuousStand].ContinuousInit.Mode, (SensorMode)IMUSet);//開始連續步態
                }
                BasketInfo->PreRotateFlag = true;
                BasketInfo->RobotPosition = BigGOAhead;
                std::printf("\033[0;33mBall at front side\033[0m\n");             
                if(BasketInfo->HorizontalHeadPosition >= (2048 + 120))//2168
                {
                    std::printf("\033[0;33mBall at left side\033[0m\n");
                    MoveHead(HeadMotorID::HorizontalID, 2048, 500);
                    tool->Delay(1500);
                    ros::spinOnce();
                    gettimeofday(&tstart, NULL);
                    gettimeofday(&tend, NULL);
                    timeuse = (1000000*(tend.tv_sec - tstart.tv_sec) + (tend.tv_usec - tstart.tv_usec))/1000;
                    while (timeuse <= 10000)
                    {
                        image();
                        ROS_INFO("Ball.X = %d", BasketInfo->Ball.X);
                        if((BasketInfo->Ball.X >= 160  && BasketInfo->Ball.X != 0) || !strategy_info->getStrategyStart())
                        {
                            break;
                        }
                        MoveContinuous(ContinuousTurnLeft);
                        gettimeofday(&tend, NULL);
                        timeuse = (1000000*(tend.tv_sec - tstart.tv_sec) + (tend.tv_usec - tstart.tv_usec))/1000;
                    }
                    BasketInfo->RobotPosition = TurnLeft;                  
                }
                else if(BasketInfo->HorizontalHeadPosition <= (2048 - 120))//1928
                {
                    std::printf("\033[0;33mBall at right side\033[0m\n");   
                    MoveHead(HeadMotorID::HorizontalID, 2048, 500);
                    tool->Delay(1500);
                    ros::spinOnce();
                    gettimeofday(&tstart, NULL);
                    gettimeofday(&tend, NULL);
                    timeuse = (1000000*(tend.tv_sec - tstart.tv_sec) + (tend.tv_usec - tstart.tv_usec))/1000;
                    while (timeuse <= 10000)
                    {
                        image();
                        ROS_INFO("Ball.X = %d", BasketInfo->Ball.X);
                        if((BasketInfo->Ball.X <= 160  && BasketInfo->Ball.X != 0) || !strategy_info->getStrategyStart())
                        {
                            break;
                        }
                        MoveContinuous(ContinuousTurnRight);
                        gettimeofday(&tend, NULL);
                        timeuse = (1000000*(tend.tv_sec - tstart.tv_sec) + (tend.tv_usec - tstart.tv_usec))/1000;
                    }
                    BasketInfo->RobotPosition = TurnRight;
                }
                if(BasketInfo->VerticalHeadPosition <= (BasketInfo->ContinuousSlowLine + 100))//catchfarside
                {
                    BasketInfo->ContinuousSlowLine = BasketInfo->ContinuousSlowLine + 100;
                    std::printf("\033[0;33mBall at close side\033[0m\n");
                }  
            }
            if(BasketInfo->ReStartFindBallFlag)
            {
                if(BasketInfo->VerticalHeadPosition <= BasketInfo->CatchBallLine && BasketInfo->StoopFlag)
                {
                    ROS_INFO("Restart");
                    walk_con->stopContinuous();
                    tool->Delay(1500);
                    BasketInfo->Robot_State = Find_Ball;
                }
                else
                {
                    BasketInfo->Robot_State = Goto_Ball;
                }
                BasketInfo->ReStartFindBallFlag = false;
            } 
            else
            {
                BasketInfo->Robot_State = Goto_Ball;
            }
        }
        else
        {
            if(BasketInfo->HorizontalHeadPosition < BasketInfo->HorizontalMinAngle + 200 || BasketInfo->HorizontalHeadPosition > BasketInfo-> HorizontalMaxAngle - 200)//當轉超過MinAngle時，將頭轉回MinAngle
            {
                walk_con->stopContinuous();
                tool->Delay(1500);
                BasketInfo->StraightCatchFlag = true;
                BasketInfo->PreRotateFlag = false;
                BasketInfo->Robot_State = Find_Ball;
            }
            if(BasketInfo->VerticalHeadPosition < BasketInfo->VerticalMinAngle)//同上概念
            {
                MoveHead(HeadMotorID::VerticalID, BasketInfo->VerticalMinAngle, 200);
                BasketInfo->Robot_State = Find_Ball;
            }
            else if(BasketInfo->VerticalHeadPosition > BasketInfo->VerticalMaxAngle)//同上概念
            {
                MoveHead(HeadMotorID::VerticalID, BasketInfo->VerticalMaxAngle, 200);
                BasketInfo->Robot_State = Find_Ball;
            }
        }
    }
}

void KidsizeStrategy::TraceballBody()
{
    if(BasketInfo->ContinuousFlag)
    {
        ROS_INFO("Catch Ball VerticalHeadPosition = %d", BasketInfo->VerticalHeadPosition);
        if(BasketInfo->Ball.size <= Ballfarsize)//漏球時，切回Find_Ball
        {
            ROS_INFO("Miss Ball");
            walk_con->stopContinuous();
            tool->Delay(1500);
            BasketInfo->StraightCatchFlag = true;
            BasketInfo->PreRotateFlag = false;
            BasketInfo->Robot_State = Find_Ball;
        }
        else if(BasketInfo->VerticalHeadPosition > BasketInfo->ContinuousSlowLine)//ContinuousStopLine是連續步態停止線
        {
            ROS_INFO("Stand_1");
            if(!walk_con->isStartContinuous())//開啟連續步態
            {
                walk_con->startContinuous((WalkingMode)BasketInfo->ContinuousStep[ContinuousStand].ContinuousInit.Mode, (SensorMode)IMUSet);//開始連續步態
            }
            else if(BasketInfo->HorizontalHeadPosition > (2048 + 130))//在這兩個區間作修正
            {
                std::printf("\033[0;33mTurn Left\033[0m\n");
                MoveContinuous(ContinuousSmallLeft);
            }
            else if(BasketInfo->HorizontalHeadPosition < (2048 - 130))//在這兩個區間作修正
            {
                std::printf("\033[0;33mTurn Right\033[0m\n");
                MoveContinuous(ContinuousSmallRight);
            }
            else
            {
                std::printf("\033[0;33mGo Ahead\033[0m\n");
                MoveContinuous(ContinuousForward);
            }
            BasketInfo->Robot_State = Trace_Ball;
        }
		else if(BasketInfo->VerticalHeadPosition <= BasketInfo->ContinuousSlowLine)
		{
            ROS_INFO("Stand_2");
            if(BasketInfo->VerticalHeadPosition <= BasketInfo->CatchBallLine)
            {
                if(walk_con->isStartContinuous())//當要回到找球狀態時，關閉連續步態
                {
                    walk_con->stopContinuous();
                    tool->Delay(1500);
                }
                BasketInfo->ContinuousFlag = false;
                BasketInfo->StoopFlag = true;
            }
            else if(!walk_con->isStartContinuous())//開啟連續步態
            {
                walk_con->startContinuous((WalkingMode)BasketInfo->ContinuousStep[ContinuousStand].ContinuousInit.Mode, (SensorMode)IMUSet);//開始連續步態
            }
            else if(BasketInfo->HorizontalHeadPosition > (2048 + 60))//
            {
                ROS_INFO("Catch Ball Left");
                MoveContinuous(ContinuousSmallTurnLeft);
                BasketInfo->Robot_State = Trace_Ball; 
            }
            else if(BasketInfo->HorizontalHeadPosition < (2048 - 60))//1948+5
            {
                ROS_INFO("Catch Ball Right");
                MoveContinuous(ContinuousSmallTurnRight);
                BasketInfo->Robot_State = Trace_Ball; 
            } 
            else
            {
                ROS_INFO("Catch Ball Small Foward");
                MoveContinuous(ContinuousSmallForward);
                BasketInfo->Robot_State = Trace_Ball; 
            }
		}
	}
    else if(BasketInfo->StoopFlag)
    { 
        if(BasketInfo->Ball.size <= Ballfarsize)//沒找到球時切回Find_Ball
        {
            ROS_INFO("Miss Ball");
            BasketInfo->Robot_State = Find_Ball;
        }
        else
        { 
            if(abs(BasketInfo->Ball.X - BasketInfo->BallVerticalBaseLine) <= BasketInfo->BallVerticalError && abs(BasketInfo->Ball.Y - BasketInfo->BallHorizontalBaseLine) <= BasketInfo->BallHorizontalError)
            {
                MoveHead(HeadMotorID::VerticalID, BasketInfo->CatchBallVerticalHeadPosition, 200);
                MoveHead(HeadMotorID::HorizontalID, 2048, 200);
                tool->Delay(1000);
                ROS_INFO("Waistdown");
                ros_com->sendBodySector(BB_WaistDown);
                tool->Delay(7000);
                BasketInfo->StoopFlag = false;
                BasketInfo->MoveFlag = true;
            }
            else
            {
                BasketInfo->Robot_State = Trace_Ball;
            }
        }
    }
    else if (BasketInfo->MoveFlag)
    {
        ROS_INFO("Push ball");
        ROS_INFO("BasketInfo->Ball.Y = %d", BasketInfo->Ball.Y);
        if(BasketInfo->Ball.Y > BasketInfo->CatchBallYLine)//155   
        {
            BasketInfo->count = (BasketInfo->Ball.Y - BasketInfo->CatchBallYLine);
            BasketInfo->HandMove = (BasketInfo->count)*2;//R3:1.75    R1:1.6//r
            BasketInfo->countdown = 1;
            ROS_INFO("IN");
            ros_com->sendSingleMotor(5, (-1)*BasketInfo->HandMove, 100);
            tool->Delay(1000);
            ros_com->sendSingleMotor(1, (1)*BasketInfo->HandMove, 100);
            tool->Delay(1000);
        }
        else if(BasketInfo->Ball.Y <= BasketInfo->CatchBallYLine)
        {
            BasketInfo->count = (BasketInfo->CatchBallYLine - BasketInfo->Ball.Y);
            BasketInfo->HandMove = (BasketInfo->count)*2;
            BasketInfo->countup = 1;
            ROS_INFO("OUT");
            ros_com->sendSingleMotor(5, (1)*BasketInfo->HandMove, 100);
            tool->Delay(1000);
            ros_com->sendSingleMotor(1, (-1)*BasketInfo->HandMove, 100);
            tool->Delay(1000);
        }
        BasketInfo->MoveFlag = false;
        BasketInfo->GetballFlag = true;
    }
    else if(BasketInfo->GetballFlag)
    {
        ros_com->sendBodySector(BB_WaistCatch);
        tool->Delay(1500);
        ROS_INFO("Waist up");
        ros_com->sendBodySector(BB_WaistUp);
        tool->Delay(8500);
        MoveHead(HeadMotorID::VerticalID, 2048, 200);
        MoveHead(HeadMotorID::HorizontalID, 2048, 200);
        if(BasketInfo->countdown)
        {
            ros_com->sendSingleMotor(5, (1)*BasketInfo->HandMove, 100);//在夾球期間，手有往內or往外，所以在投球前要將其歸位
            tool->Delay(1000);
            ros_com->sendSingleMotor(1, (-1)*BasketInfo->HandMove, 100);//在夾球期間，手有往內or往外，所以在投球前要將其歸位
            tool->Delay(1000);
            BasketInfo->countdown = 0;
        }
        else if(BasketInfo->countup)
        {
            ros_com->sendSingleMotor(5, (-1)*BasketInfo->HandMove, 100);//在夾球期間，手有往內or往外，所以在投球前要將其歸位
            tool->Delay(1000);
            ros_com->sendSingleMotor(1, (1)*BasketInfo->HandMove, 100);//在夾球期間，手有往內or往外，所以在投球前要將其歸位
            tool->Delay(1000);
            BasketInfo->countup = 0;
        }
        ROS_INFO("Hands Back");
        tool->Delay(1000);
        ros_com->sendBodySector(BB_WaistUpFeedBack);
        tool->Delay(2000);
        if(!walk_con->isStartContinuous())
        {
            walk_con->startContinuous((WalkingMode)BasketInfo->ContinuousStep[ContinuousStand].ContinuousInit.Mode, (SensorMode)IMUSet);
        }
        BasketInfo->GetballFlag = false;
        BasketInfo->TurnFlag = true;
	}
    else if(BasketInfo->TurnFlag)
    {
		if(abs(BasketInfo->Basket.X - 160) <= 90 && strategy_info->getIMUValue().Yaw < 60 && strategy_info->getIMUValue().Yaw > -60)//看到籃框時進到Find_Target
		{
            ROS_INFO("Start Finding Basket");
            tool->Delay(200);
            BasketInfo->Robot_State = Find_Target;
		}
        if(BasketInfo->RobotPosition == TurnLeft)//根據機器人一開始的方向來判別現在該轉哪邊
        {
            ROS_INFO("IMU Value = %f", strategy_info->getIMUValue().Yaw);
            if(!BasketInfo->FaceBasketFlag)
            {
                if(strategy_info->getIMUValue().Yaw < -30)
                {
                    ROS_INFO("Turn Left IMU");
                     BasketInfo->RobotPosition = TurnRight;
                }
                BasketInfo->FaceBasketFlag = true;
            }
            else
            { 
                ROS_INFO("Turn Right to Find Basket");
                MoveContinuous(ContinuousTurnRight);
            }
        }
        else if(BasketInfo->RobotPosition == TurnRight)
        {
            ROS_INFO("IMU Value = %f", strategy_info->getIMUValue().Yaw);
            if(!BasketInfo->FaceBasketFlag)
            {
                if(strategy_info->getIMUValue().Yaw > 30)
                {
                    ROS_INFO("Turn Right IMU");
                    BasketInfo->RobotPosition = TurnLeft;
                }
                BasketInfo->FaceBasketFlag = true;
            }
            else
            { 
                ROS_INFO("Turn Left to Find Basket ");
                MoveContinuous(ContinuousTurnLeft);
            }
        }
        else if(BasketInfo->RobotPosition == BigGOAhead)
        {
            if(strategy_info->getIMUValue().Yaw > 0)
            {
                ROS_INFO("Forward Turn Right");
                MoveContinuous(ContinuousTurnRight);
            }
            else if(strategy_info->getIMUValue().Yaw <= 0)
            {
                ROS_INFO("Forward Turn left");
                MoveContinuous(ContinuousTurnLeft);
            }
        }
    }
}

void KidsizeStrategy::FindbasketHead()//跟FindballHead()相同概念
{
    if(walk_con->isStartContinuous())
    {
        ros::spinOnce();
        gettimeofday(&tstart, NULL);
        gettimeofday(&tend, NULL);
        timeuse = (1000000*(tend.tv_sec - tstart.tv_sec) + (tend.tv_usec - tstart.tv_usec))/1000;
        while (timeuse <= 500)
        {
            MoveContinuous(ContinuousStay);
            ros::spinOnce();
            gettimeofday(&tend, NULL);
            timeuse = (1000000*(tend.tv_sec - tstart.tv_sec) + (tend.tv_usec - tstart.tv_usec))/1000;
        }
        walk_con->stopContinuous();
        tool->Delay(3000);//1500+1500
    }
    if(BasketInfo->Basket.size > Basketfarsize)
	{
		ROS_INFO("Found Basket");
        BasketInfo->Robot_State = Trace_Target;
	}
    else
    {
        switch(BasketInfo->HeadTurnSide)
        {
            case HaedTurnRight:
                if ((BasketInfo->HorizontalHeadPosition - BasketInfo->HeadTurnSpeed) > BasketInfo->BasketHorizontalMinAngle)
                {
                    MoveHead(HeadMotorID::HorizontalID, BasketInfo->HorizontalHeadPosition - BasketInfo->HeadTurnSpeed, 200);
                }
                else if ((BasketInfo->HorizontalHeadPosition - BasketInfo->HeadTurnSpeed) <= BasketInfo->BasketHorizontalMinAngle)
                {
                    BasketInfo->HeadTurnSide = HeadTurnLeft;
                    BasketInfo->Head = (BasketInfo->Head+1)%2;
                }
                break;
            case HeadTurnLeft:
                if ((BasketInfo->HorizontalHeadPosition + BasketInfo->HeadTurnSpeed) < BasketInfo->BasketHorizontalMaxAngle)
                {
                    MoveHead(HeadMotorID::HorizontalID, BasketInfo->HorizontalHeadPosition + BasketInfo->HeadTurnSpeed, 200);
                }
                else if((BasketInfo->HorizontalHeadPosition + BasketInfo->HeadTurnSpeed) >= BasketInfo->BasketHorizontalMaxAngle)
                {
                    BasketInfo->HeadTurnSide = HaedTurnRight;
                    BasketInfo->Head = (BasketInfo->Head+1)%2;
                }
                break;
        }
    }	
}

void KidsizeStrategy::TracebasketHead()
{
    if(BasketInfo->LayUpFlag)//上籃策略
    {
        if(BasketInfo->Basket.size <= Basketfarsize)
        {
            ROS_INFO("miss the basket");
            BasketInfo->Robot_State = Find_Target;
        }
        else
        {
            BasketInfo->BasketMoveX = BasketInfo->Basket.X - 160;//可以當作與籃框baseline的差
            BasketInfo->BasketMoveY = BasketInfo->Basket.Y - 120;
            BasketInfo->ErrorHorizontalAngle = BasketInfo->ImgHorizontalAngle * (double)BasketInfo->BasketMoveX / (double)RobotVisionWidth;//馬達轉攝影機320pixel時轉的角度*與球baseline的差/320pixel,算出會得到角度
            BasketInfo->ErrorVerticalAngle = BasketInfo->ImgVerticalAngle * (double)BasketInfo->BasketMoveY / (double)RobotVisionHeight;//馬達轉攝影機240pixel時轉的角度*與球baseline的差/240pixel,算出會得到角度
            MoveHead(HeadMotorID::HorizontalID, BasketInfo->HorizontalHeadPosition - (MotorMoveOneDegree * BasketInfo->ErrorHorizontalAngle *TraceDegreePercent*1), 200);//再利用上面得到的角度來換算成刻度，來call   MoveHead()
            MoveHead(HeadMotorID::VerticalID, BasketInfo->VerticalHeadPosition - (MotorMoveOneDegree * BasketInfo->ErrorVerticalAngle * TraceDegreePercent*1), 200);
            BasketInfo->Robot_State = UP_Basket;
        }
	}
    else//投籃策略，這邊trace的概念跟traceball的概念一樣
    {
        if(!walk_con->isStartContinuous())//開啟連續步態
        {
            walk_con->startContinuous((WalkingMode)BasketInfo->ContinuousStep[ContinuousStand].ContinuousInit.Mode, (SensorMode)IMUSet);//開始連續步態  
        }
        SelectBaseLine();

        if(BasketInfo->HorizontalHeadPosition >= (2048 - 10) && BasketInfo->HorizontalHeadPosition <= (2048 + 10) && BasketInfo->Basket.size >= BasketInfo->SizeOfDist[1] && BasketInfo->Basket.size <= (BasketInfo->SizeOfDist[1]+BasketInfo->SizeOfDist[0])/2)
        {
            BasketInfo->Robot_State = Goto_Target;
        }
        else if(BasketInfo->Basket.size <= Basketfarsize)
        {
            ROS_INFO("Miss Basket");
            BasketInfo->Robot_State = Find_Target;
        }
        else
        {
            BasketInfo->BasketMoveX = (BasketInfo->Basket.X - BasketInfo->BasketVerticalBaseLine);
            BasketInfo->BasketMoveY = (BasketInfo->Basket.Y - BasketInfo->BasketHorizontalBaseLine);
            BasketInfo->ErrorHorizontalAngle = BasketInfo->ImgHorizontalAngle * (double)BasketInfo->BasketMoveX/(double)RobotVisionWidth;
            BasketInfo->ErrorVerticalAngle  = BasketInfo->ImgVerticalAngle * (double)BasketInfo->BasketMoveY/(double)RobotVisionHeight;
            MoveHead(HeadMotorID::HorizontalID, BasketInfo->HorizontalHeadPosition - (MotorMoveOneDegree * BasketInfo->ErrorHorizontalAngle * TraceDegreePercent*0.5) , 200);
            MoveHead(HeadMotorID::VerticalID, BasketInfo->VerticalHeadPosition - (MotorMoveOneDegree * BasketInfo->ErrorVerticalAngle * TraceDegreePercent*0.5) , 200);
            
            ROS_INFO("Adjust direction 1");
            ROS_INFO("Basket.X = %d", BasketInfo->Basket.X);
            if(BasketInfo->Basket.size < BasketInfo->SizeOfDist[1])
            {
                ROS_INFO("Forward");
                MoveContinuous(ContinuousSmallForward);
            }
            else if(BasketInfo->Basket.size > (BasketInfo->SizeOfDist[1]+BasketInfo->SizeOfDist[0])/2)
            {
                ROS_INFO("Back");
                MoveContinuous(ContinuousBackward);
            }
            else if(BasketInfo->HorizontalHeadPosition > (2048 + 10))
            {
                ROS_INFO("Turn Left");
                MoveContinuous(ContinuousSmallLeft);
            }
            else if(BasketInfo->HorizontalHeadPosition < (2048 - 10))
            {
                ROS_INFO("Turn Right");
                MoveContinuous(ContinuousSmallTurnRight);
            }
            /*if(BasketInfo->VerticalHeadPosition < BasketInfo->BasketVerticalMinAngle)
            {
                MoveHead(HeadMotorID::VerticalID, BasketInfo->BasketVerticalMinAngle, 200);
            }
            else if(BasketInfo->VerticalHeadPosition > BasketInfo->BasketVerticalMaxAngle)
            {
                MoveHead(HeadMotorID::VerticalID, BasketInfo->BasketVerticalMaxAngle, 200);
            } */
        }
    }
}

void KidsizeStrategy::TracebasketBody()
{
    tool->Delay(1000);
    SelectBaseLine();
	if(BasketInfo->RoateFlag)
	{
        ROS_INFO("Adjust direction 2");
        ROS_INFO("HorizontalHeadPosition = %d", BasketInfo->HorizontalHeadPosition);
        if(BasketInfo->HorizontalHeadPosition >= (2048 - 10) && BasketInfo->HorizontalHeadPosition <= (2048 + 10))
		{
            ROS_INFO("Body aimed basket");
            if(walk_con->isStartContinuous())//當要回到找球狀態時，關閉連續步態
            {
                walk_con->stopContinuous();
                tool->Delay(1500);
            }           
            MoveHead(HeadMotorID::VerticalID, 1990, 200);
            BasketInfo->DistanceError = strategy_info->getIMUValue().Yaw * BasketInfo->DistanceErrorCount;
            BasketInfo->RoateFlag = false;
            BasketInfo->WaistFlag = true;
		}
		else
		{
			BasketInfo->Robot_State = Trace_Target;
		}
	}
	else if (BasketInfo->WaistFlag)
	{
		/*
        if(BasketInfo->Basket.X > 160)
        {
            BasketInfo->Distancenew = CompensateDistant() + BasketInfo->Error ;//+ (BasketInfo->Basket.X - 160)/24;
        }
        else if(BasketInfo->Basket.X <= 160)
        {
            BasketInfo->Distancenew = CompensateDistant() + BasketInfo->Error;// + (160 - BasketInfo->Basket.X)/48;
        }
		*/
		HeadMeasureDistance();
        SelectBaseLine();//根據測距得到的值來選擇要哪條baseline
        ROS_INFO("1.Distancenew : %f, BaseLine = %d", BasketInfo->Distancenew, BasketInfo->BasketVerticalBaseLine);
        ROS_INFO("BasketInfo->Basket.X = %d", BasketInfo->Basket.X);
        if((BasketInfo->Basket.X - BasketInfo->BasketVerticalBaseLine) > 0)//轉腰調整Basket.X與BasketVerticalBaseLine的誤差
		{
            ROS_INFO("RIGHT");
			ros_com->sendSingleMotor(9, (-1)*(BasketInfo->Basket.X - BasketInfo->BasketVerticalBaseLine), 100);

		}
		else if((BasketInfo->Basket.X - BasketInfo->BasketVerticalBaseLine) < 0)
		{
            ROS_INFO("LEFT");
			ros_com->sendSingleMotor(9, BasketInfo->BasketVerticalBaseLine - BasketInfo->Basket.X, 100);
		}  
        tool->Delay(1000);
        
		//HeadMeasureDistance();
        SelectBaseLine();
        ROS_INFO("2.Distancenew : %f, BaseLine = %d", BasketInfo->Distancenew, BasketInfo->BasketVerticalBaseLine);
        ROS_INFO("-------------------------------------------------");
        if(abs(BasketInfo->Basket.X - BasketInfo->BasketVerticalBaseLine) < BasketInfo->WaistError)//當Basket.X與BasketVerticalBaseLine的誤差小於WaistError
        {
            ROS_INFO("Align the basketline %d", BasketInfo->BasketVerticalBaseLine);
            BasketInfo->WaistFlag = false;
            BasketInfo->ComputeFlag = true;
        }
    }
    else if(BasketInfo->ComputeFlag)
    {
		ROS_INFO("CompteFuzzy");
        HeadMeasureDistance();
		ComputeFuzzy();
        if(BasketInfo->ReaimFlag)
        {
            SelectBaseLine();
            ROS_INFO("Reaimed BasketVerticalBaseLine = %d", BasketInfo->BasketVerticalBaseLine);
            while(abs(BasketInfo->Basket.X - BasketInfo->BasketVerticalBaseLine) > BasketInfo->WaistError)//當轉完腰後誤差大於WaistError時，就重新再轉一次
            {
                if((BasketInfo->Basket.X - BasketInfo->BasketVerticalBaseLine) > 0)
                {
                    ROS_INFO("RIGHT");
                    ros_com->sendSingleMotor(9, (-1)*7, 100);
                }
                else if((BasketInfo->Basket.X - BasketInfo->BasketVerticalBaseLine) < 0)
                {
                    ROS_INFO("LEFT");
                    ros_com->sendSingleMotor(9, 5, 100);
                }
                image();      
            }
            BasketInfo->ReaimFlag = true;
        }

        ros_com->sendHandSpeed(BB_ShootingBall, BasketInfo->disspeed);//根據ComputeFuzzy()得到的disspeed來送速度到1116磁區
        ROS_INFO("BasketInfo->disspeed %d", BasketInfo->disspeed);

        BasketInfo->ComputeFlag = false;
        BasketInfo->RaiseFlag = true;
    }
	else if (BasketInfo->RaiseFlag)
	{
        ROS_INFO("Ready to shoot!!");
		ros_com->sendBodySector(BB_RaiseHand);//舉手
		tool->Delay(4500);
		BasketInfo->RaiseFlag = false;
		BasketInfo->ThrowBallFlag = true;
	}
	else if (BasketInfo->ThrowBallFlag)
	{
        ROS_INFO("Shoot!!");
		ros_com->sendBodySector(BB_ShootingBall);//射出去
		tool->Delay(2000);
        
		BasketInfo->ThrowBallFlag = false;         
		BasketInfo->Robot_State = End;

        std::printf("\033[0;33mDistancenew : %f\033[0m\n", BasketInfo->Distancenew);
        std::printf("\033[0;33mDisspeed : %d\033[0m\n", BasketInfo->disspeed);
        std::printf("\033[0;33mBaseLine : %d\033[0m\n", BasketInfo->BasketVerticalBaseLine);

        ROS_INFO("END");
	}
}

void KidsizeStrategy::UPbasket()
{   
    ROS_INFO("VerticalHeadPosition = %d", BasketInfo->VerticalHeadPosition);
    if (BasketInfo->VerticalHeadPosition > BasketInfo->UpBasketStopLine)//利用籃框底部的位置來判別何時停止;將YAMX改變成頭部垂直刻度
	{        
		if (!walk_con->isStartContinuous())
		{ 
            if(!BasketInfo->LeftHandUpFlag)//將左手抬起，避免壓到球柱
            {
                ros_com->sendSingleMotor(4, 1024, 100);
                tool->Delay(1500);
                BasketInfo->LeftHandUpFlag = true;  
            }
			walk_con->startContinuous((WalkingMode)BasketInfo->ContinuousStep[ContinuousStand].ContinuousInit.Mode, (SensorMode)IMUSet);//開始連續步態
            ros::spinOnce();
            gettimeofday(&tstart, NULL);
            gettimeofday(&tend, NULL);
            timeuse = (1000000*(tend.tv_sec - tstart.tv_sec) + (tend.tv_usec - tstart.tv_usec))/1000;
            while (timeuse <= 1000)
            {
                MoveContinuous(ContinuousStay);
                ros::spinOnce();
                gettimeofday(&tend, NULL);
                timeuse = (1000000*(tend.tv_sec - tstart.tv_sec) + (tend.tv_usec - tstart.tv_usec))/1000;
            }
		}
		else if(BasketInfo->HorizontalHeadPosition < (2048 - 100))//連續步態期間的右旋修正
        {
            MoveContinuous(ContinuousFastBigRight);
        } 
        else if(BasketInfo->HorizontalHeadPosition > (2048 + 100))
        {
            MoveContinuous(ContinuousFastBigLeft);
        }
        else if(BasketInfo->HorizontalHeadPosition < (2048 - 20))
        {
            MoveContinuous(ContinuousFastSmallRight);
        }
        else if(BasketInfo->HorizontalHeadPosition > (2048 + 20))
        {
            MoveContinuous(ContinuousFastSmallLeft);
        }
        else
        {  
            MoveContinuous(ContinuousFastForward);
        }       
        BasketInfo->Robot_State = Trace_Target;
	}
    else  //當Y.Max>230時 強制停止連續步態
    {
        if (walk_con->isStartContinuous())
        {
            walk_con->stopContinuous();
            tool->Delay(1500);  
            BasketInfo->Robot_State = Slum_Dunk_Basket;
        }
		else
		{
			BasketInfo->Robot_State = Trace_Target;
		}        
    }   
}

void KidsizeStrategy::Slum_Dunk()//灌籃
{   
    if(BasketInfo->HandUPFlag == true)
    {
        tool->Delay(3000);
        image();
        BasketInfo->BasketMoveX = BasketInfo->Basket.X - 160;//可以當作與籃框baseline的差
        BasketInfo->ErrorHorizontalAngle = BasketInfo->ImgHorizontalAngle * (double)BasketInfo->BasketMoveX / (double)RobotVisionWidth;//馬達轉攝影機320pixel時轉的角度*與球baseline的差/320pixel,算出會得到角度
        MoveHead(HeadMotorID::HorizontalID, BasketInfo->HorizontalHeadPosition - (MotorMoveOneDegree * BasketInfo->ErrorHorizontalAngle), 200);//再利用上面得到的角度來換算成刻度，來call   MoveHead()
        ROS_INFO("Hand_UP");
        ros_com->sendBodySector(BB_UpHand);
        tool->Delay(10000);
        BasketInfo->HandUPFlag = false;
        BasketInfo->SlumDunkFlag = true;
        ros_com->sendSingleMotor(9, BasketInfo->HorizontalHeadPosition - BasketInfo->SlumDunkHorizontalAngle, 50);     
        tool->Delay(2000);
    }
    else if(BasketInfo->SlumDunkFlag)
    {
        ros_com->sendBodySector(BB_SlumDunk);//灌籃!!!
        tool->Delay(5000);
        ROS_INFO("Slum Dunk Is End!!!!!!!!!!!!!!");
        std::printf("\033[0;33m VerticalHeadPosition : %d\033[0m\n", BasketInfo->VerticalHeadPosition);
        std::printf("\033[0;33m Horizontal Move Degree : %d\033[0m\n", BasketInfo->HorizontalHeadPosition - BasketInfo->SlumDunkHorizontalAngle);
        BasketInfo->SlumDunkFlag == false;
        BasketInfo->Robot_State = End;
    }
}
