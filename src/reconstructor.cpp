#include "vive_ros/reconstructor.hpp"

Reconstructor::Reconstructor() {
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "vive_reconstructor",
                  ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }
    int cameraID;
    nh_.getParam("/vive/cameraID", cameraID);
    ROS_INFO("Opening camera capture ID: %d", cameraID);
    inputCapture.open(cameraID);
    if (!inputCapture.isOpened()) {
        ROS_ERROR(
                "could not open vive camera, check reconstructor.launch for correct cameraID, and make sure you I have sufficient permissions");
    }
    inputCapture.read(img_old);
    image_size = img_old.size();

//    cvtColor(img_old, img_old_cpy, CV_BGR2GRAY);

    string filepath;
    nh_.getParam("/vive/intrinsics_path", filepath);
    cv::FileStorage fs(filepath, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        ROS_ERROR(
                "Could not read intrinsic params from %s, check the reconstructor.launch file and update the filepath!",
                filepath.c_str());
    }
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs.release();

    ROS_INFO_STREAM("\nvive intrinsic camera params:\n" << cameraMatrix << "\ndistortion coeffs:\n" << distCoeffs);

    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
                            getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, image_size, 1, image_size, 0),
                            image_size, CV_16SC2, map1, map2);

//    while(true){
//        inputCapture.read(img_new);
//        imshow("img_new", img_new);
//        Mat test = img_new.clone();
//        remap(test, test, map1, map2, INTER_LINEAR);
//        imshow("img_new_rect", test);
//        waitKey(1);
//    }

    ROS_INFO("initializing surf detector");
    detector = SURF::create(400);

    // get surf points
    detector->detect(img_old, keypoints_old);

    // get the pose
    bool got_initial_pose = false;
    while (!got_initial_pose) {
        try {
            tf_listener.lookupTransform("world_vive", "hmd",
                                        ros::Time(0), transform_old);
            got_initial_pose = true;
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
    }
    stereoBM = StereoBM::create(0, 5);

    marker_visualization_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
}

Reconstructor::~Reconstructor() {
}

void Reconstructor::Run(int rate) {
    double tf_matrix[3][4];

    ros::Rate loop_rate(rate);
    while (ros::ok()) {

        ROS_INFO_STREAM("\nvive intrinsic camera params:\n" << cameraMatrix << "\ndistortion coeffs:\n" << distCoeffs);

        try {
            tf_listener.lookupTransform("world_vive", "hmd",
                                        ros::Time(0), transform_new);
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        // get new camera image
        inputCapture.read(img_new);

        imshow("img_new", img_new);
        imshow("img_old", img_old);

        //-- Step 1: Detect the keypoints using SURF Detector
        detector->detect(img_new, keypoints_new);

        //-- Step 2: Calculate descriptors (feature vectors)
        detector->compute(img_old, keypoints_old, descriptors_old);
        detector->compute(img_new, keypoints_new, descriptors_new);

        //-- Step 3: Matching descriptor vectors using FLANN matcher
        std::vector<DMatch> matches;
        matcher.match(descriptors_old, descriptors_new, matches);

        double max_dist = 0;
        double min_dist = 100;

        //-- Quick calculation of max and min distances between keypoints
        for (int i = 0; i < descriptors_old.rows; i++) {
            double dist = matches[i].distance;
            if (dist < min_dist) min_dist = dist;
            if (dist > max_dist) max_dist = dist;
        }

        //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
        //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
        //-- small)
        //-- PS.- radiusMatch can also be used here.
        std::vector<DMatch> good_matches;

        vector<Point2f> allimgpt[2];
        for (int i = 0; i < descriptors_old.rows; i++) {
            if (matches[i].distance <= max(2 * min_dist, 0.02)) {
                good_matches.push_back(matches[i]);
                allimgpt[0].push_back(keypoints_old[matches[i].trainIdx].pt);
                allimgpt[1].push_back(keypoints_new[matches[i].queryIdx].pt);
            }
        }

        //-- Draw only "good" matches
        Mat img_matches;
        drawMatches(img_old, keypoints_old, img_new, keypoints_new,
                    good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                    vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

        //-- Show detected matches
        imshow("Good Matches", img_matches);

        // calculate transform between the two camera frames
        tf::Transform tf_between_frames = transform_old.inverseTimes(transform_new);
        Eigen::Affine3d Rt_old, Rt_new;
        tf::transformTFToEigen(transform_old, Rt_old);
        tf::transformTFToEigen(transform_new, Rt_new);
        Eigen::Matrix3d R_old = Rt_old.rotation();
        Eigen::Matrix3d R_new = Rt_new.rotation();
        Eigen::Vector3d T_old = Rt_old.translation();
        Eigen::Vector3d T_new = Rt_new.translation();
        Eigen::Matrix<double, 3, 4> RT_old, RT_new;
        RT_old.topLeftCorner(3, 3) = R_old;
        RT_new.topLeftCorner(3, 3) = R_new;
        RT_old.topRightCorner(3, 1) = T_old;
        RT_new.topRightCorner(3, 1) = T_new;
//        T_ *= 1000.0;
        ROS_INFO_STREAM("RT_old:\n" << RT_old << "\nRT_new:\n" << RT_new);
//        Mat R,T;
//        eigen2cv(R_, R);
//        eigen2cv(T_, T);
        Mat rt_old(3, 4, CV_64F), rt_new(3, 4, CV_64F);
        eigen2cv(RT_old, rt_old);
        eigen2cv(RT_new, rt_new);
//        rt_old = cameraMatrix * rt_old;
//        rt_new = cameraMatrix * rt_new;

        cv::Mat pnts3D;
        float *a = new float[good_matches.size() * 2];
        float *b = new float[good_matches.size() * 2];
        cv::Mat cam0pnts(2, 1, CV_32F, a);
        cv::Mat cam1pnts(2, 1, CV_32F, b);
        ROS_INFO_STREAM(good_matches.size() << " points");
        waitKey(1);
        if (good_matches.size() != 0){
            for (int i = 0; i < good_matches.size(); i++) {
                cam0pnts.at<Vec2f>(i) = allimgpt[0][i];
                cam1pnts.at<Vec2f>(i) = allimgpt[1][i];
                ROS_INFO_STREAM(cam0pnts.at<Vec2f>(i));
            }

            Mat cam0pnts_undistorted, cam1pnts_undistorted;
            undistortPoints(cam0pnts, cam0pnts_undistorted, cameraMatrix, distCoeffs);
            undistortPoints(cam1pnts, cam1pnts_undistorted, cameraMatrix, distCoeffs);
//
            delete[] a;
            delete[] b;
//        Mat x1(2, allimgpt[0].size(), CV_32F, &allimgpt[0][0]);
//        Mat x2(2, allimgpt[0].size(), CV_32F, &allimgpt[1][0]);
//        Mat res_;
//
//        triangulatePoints(P1, P2, x1, x2, res_);

            triangulatePoints(rt_old, rt_new, cam0pnts_undistorted, cam1pnts_undistorted, pnts3D);

//        Mat R1, R2, P1, P2, Q;

//        stereoRectify(cameraMatrix, distCoeffs, cameraMatrix, distCoeffs, image_size, R, T, R1, R2, P1, P2, Q,0,1);
//        Mat F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
//        Mat H1, H2;
//        stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, image_size, H1, H2, 3);
//
//        R1 = cameraMatrix.inv()*H1*cameraMatrix;
//        R2 = cameraMatrix.inv()*H2*cameraMatrix;
//        P1 = cameraMatrix;
//        P2 = cameraMatrix;

//        Mat disparity(img_old.size(), CV_8UC1);
//
//        cvtColor(img_new, img_new_cpy,CV_BGR2GRAY);
//
//        stereoBM->compute(img_new_cpy, img_old_cpy, disparity);
//        imshow("disparity", disparity*255.0f);

            visualization_msgs::Marker points;
            points.header.frame_id = "world";
            points.ns = "RGBD";
            points.type = visualization_msgs::Marker::POINTS;
            points.scale.x = 0.01;
            points.scale.y = 0.01;
            points.lifetime = ros::Duration();
            points.header.stamp = ros::Time::now();
            static long int counter = 0;
            points.id = counter++;
            points.pose.position.x = T_old(0);
            points.pose.position.y = T_old(1);
            points.pose.position.z = T_old(2);
            Eigen::Quaterniond q(R_old);
            points.pose.orientation.x = q.x();
            points.pose.orientation.y = q.y();
            points.pose.orientation.z = q.z();
            points.pose.orientation.w = q.w();
//        int w = disparity.cols;
//        int h = disparity.rows;
            unsigned short *rgb_ = (unsigned short *) img_old.data;

//        Mat out3d;
//        reprojectImageTo3D(disparity,out3d,Q,true,CV_32F);

//        cv::Mat_<double> vec_tmp(4,1);
//        for(int y=0; y<img_old.rows; ++y) {
//            for(int x=0; x<img_old.cols; ++x) {
//                vec_tmp(0)=x; vec_tmp(1)=y; vec_tmp(2)=(double)disparity.at<float>(y,x); vec_tmp(3)=1;
//                vec_tmp = Q*vec_tmp;
//                if(fabs(vec_tmp(3))>0)
//                    vec_tmp /= vec_tmp(3);
//                else
//                    continue;
            for (int i = 0; i < good_matches.size(); i++) {
                Vec3f point = pnts3D.at<Vec3f>(i);
                if (norm(point) > 1000)
                    continue;
                geometry_msgs::Point p;
                p.x = point(0);
                p.y = point(1);
                p.z = point(2);
                points.points.push_back(p);
                std_msgs::ColorRGBA c;
//                c.b = rgb_[0 + 3 * (x + w * y)];
//                c.g = rgb_[1 + 3 * (x + w * y)];
//                c.r = rgb_[2 + 3 * (x + w * y)];
                c.b = 1.0;
                c.g = 1.0;
                c.r = 1.0;
                c.a = 1.0;
                points.colors.push_back(c);
            }
//        }
            marker_visualization_pub.publish(points);
//
//        Mat rmap[2][2];
//        //Precompute maps for cv::remap()
//        initUndistortRectifyMap(cameraMatrix, distCoeffs, R1, P1, image_size, CV_16SC2, rmap[0][0], rmap[0][1]);
//        initUndistortRectifyMap(cameraMatrix, distCoeffs, R2, P2, image_size, CV_16SC2, rmap[1][0], rmap[1][1]);
//        {
//            Mat canvas;
//            double sf;
//            int w, h;
//            sf = 600. / MAX(image_size.width, image_size.height);
//            w = cvRound(image_size.width * sf);
//            h = cvRound(image_size.height * sf);
//            canvas.create(h, w * 2, CV_8UC3);
//
//            for (int k = 0; k < 2; k++) {
//                Mat rimg, cimg;
//                if (k == 0)
//                    remap(img_old, rimg, rmap[k][0], rmap[k][1], INTER_LINEAR);
//                else
//                    remap(img_new, rimg, rmap[k][0], rmap[k][1], INTER_LINEAR);
//                Mat canvasPart = canvas(Rect(w * k, 0, w, h));
//                resize(rimg, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
//            }
//
//            for (int j = 0; j < canvas.rows; j += 16)
//                line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
//            imshow("rectified", canvas);
//        }
        }
        waitKey(1);

        loop_rate.sleep();

        ros::spinOnce();

        std::swap(img_old, img_new);
        std::swap(img_old_cpy, img_new_cpy);
        std::swap(keypoints_old, keypoints_new);
        std::swap(descriptors_old, descriptors_new);
        std::swap(transform_old, transform_new);
    }
}

// Main
int main(int argc, char **argv) {
    ros::init(argc, argv, "reconstruct_node");

    Reconstructor reconstructor;

    reconstructor.Run(5);

    return 0;
};
