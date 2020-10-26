#include <iostream>
#include <pylon/PylonIncludes.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <coordinates.h>


std::pair<cv::Mat,cv::Mat> findCalibrationVariables(std::string path);

int main()
{
    int myExposure = 30000;
    std::pair<cv::Mat,cv::Mat> calibrationMaps;
    calibrationMaps = findCalibrationVariables("../Images/*.bmp");

    // The exit code of the sample application.
    int exitCode = 0;

    // Automagically call PylonInitialize and PylonTerminate to ensure the pylon runtime system
    // is initialized during the lifetime of this object.
    Pylon::PylonAutoInitTerm autoInitTerm;

    try
    {
        // Create an instant camera object with the camera device found first.
        Pylon::CInstantCamera camera( Pylon::CTlFactory::GetInstance().CreateFirstDevice());

        // Get a camera nodemap in order to access camera parameters.
        GenApi::INodeMap& nodemap= camera.GetNodeMap();

        // Open the camera before accessing any parameters.
        camera.Open();
        // Create pointers to access the camera Width and Height parameters.
        GenApi::CIntegerPtr width= nodemap.GetNode("Width");
        GenApi::CIntegerPtr height= nodemap.GetNode("Height");

        // The parameter MaxNumBuffer can be used to control the count of buffers
        // allocated for grabbing. The default value of this parameter is 10.
        //camera.MaxNumBuffer = 5;

        // Create a pylon ImageFormatConverter object.
        Pylon::CImageFormatConverter formatConverter;
        // Specify the output pixel format.
        formatConverter.OutputPixelFormat= Pylon::PixelType_BGR8packed;
        // Create a PylonImage that will be used to create OpenCV images later.
        Pylon::CPylonImage pylonImage;

        // Create an OpenCV image.
        cv::Mat openCvImage;


        // Set exposure to manual
        GenApi::CEnumerationPtr exposureAuto( nodemap.GetNode( "ExposureAuto"));
        if ( GenApi::IsWritable( exposureAuto)){
            exposureAuto->FromString("Off");
            std::cout << "Exposure auto disabled." << std::endl;
        }

        // Set custom exposure
        GenApi::CFloatPtr exposureTime = nodemap.GetNode("ExposureTime");
        std::cout << "Old exposure: " << exposureTime->GetValue() << std::endl;
        if(exposureTime.IsValid()) {
            if(myExposure >= exposureTime->GetMin() && myExposure <= exposureTime->GetMax()) {
                exposureTime->SetValue(myExposure);
            }else {
                exposureTime->SetValue(exposureTime->GetMin());
                std::cout << ">> Exposure has been set with the minimum available value." << std::endl;
                std::cout << ">> The available exposure range is [" << exposureTime->GetMin() << " - " << exposureTime->GetMax() << "] (us)" << std::endl;
            }
        }else {

            std::cout << ">> Failed to set exposure value." << std::endl;
            return false;
        }
        std::cout << "New exposure: " << exposureTime->GetValue() << std::endl;

        // Start the grabbing of c_countOfImagesToGrab images.
        // The camera device is parameterized with a default configuration which
        // sets up free-running continuous acquisition.
        camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);

        // This smart pointer will receive the grab result data.
        Pylon::CGrabResultPtr ptrGrabResult;

        // image grabbing loop
        int frame = 1;
        while ( camera.IsGrabbing())
        {
            // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
            camera.RetrieveResult( 5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);

            // Image grabbed successfully?
            if (ptrGrabResult->GrabSucceeded())
            {
                // Access the image data.
                //cout << "SizeX: " << ptrGrabResult->GetWidth() << endl;
                //cout << "SizeY: " << ptrGrabResult->GetHeight() << endl;

                // Convert the grabbed buffer to a pylon image.
                formatConverter.Convert(pylonImage, ptrGrabResult);

                // Create an OpenCV image from a pylon image.
                openCvImage= cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *) pylonImage.GetBuffer());
                cv::remap(openCvImage, openCvImage, calibrationMaps.first, calibrationMaps.second, CV_INTER_LINEAR);

                //////////////////////////////////////////////////////
                //////////// Here your code begins ///////////////////
                //////////////////////////////////////////////////////

                cv::Mat ball_ROI(openCvImage, cv::Rect(430,0,700, 880));
                cv::Mat grey(ball_ROI.clone());
                cv::cvtColor(ball_ROI, grey, cv::COLOR_BGR2GRAY);

                std::vector<cv::Vec3f> circles;
                cv::GaussianBlur(grey, grey, cv::Size(5,5), 0);
                cv::HoughCircles(grey, circles, CV_HOUGH_GRADIENT, 2, 25, 300, 50, 15, 20);


                for( size_t i = 0; i < circles.size(); i++ )
                {
                    cv::Vec3i c = circles[i];
                    cv::Point center = cv::Point(c[0], c[1]);
                    // circle center
                    cv::circle( ball_ROI, center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA);
                    // circle outline
                    int radius = c[2];
                    cv::circle( ball_ROI, center, radius, cv::Scalar(255,0,255), 3, cv::LINE_AA);
                }

//                cv::Mat image_red = cv::Mat(openCvImage.rows, openCvImage.cols, openCvImage.type(), cv::Scalar( 0, 0, 0 ));

//                for( int i = 0; i < openCvImage.cols; i++ ) {
//                    for( int j = 0; j < openCvImage.rows; j++ ) {
//                        if( openCvImage.at< cv::Vec3b >(j,i)[2]-(openCvImage.at< cv::Vec3b >(j,i)[1] + openCvImage.at< cv::Vec3b >(j,i)[0]) > 30  ) {
//                            image_red.at< cv::Vec3b >(j,i) = openCvImage.at< cv::Vec3b >(j,i);
//                        }
//                    }
//                }
                // Create an OpenCV display windo0w
                cv::namedWindow( "myWindow", CV_WINDOW_NORMAL); // other options: CV_AUTOSIZE, CV_FREERATIO

                // Display the current image in the OpenCV display window.
                cv::imshow( "myWindow", ball_ROI);

                // Detect key press and quit if 'q' is pressed
                int keyPressed = cv::waitKey(1);
                if(keyPressed == 'q'){ //quit
                    std::cout << "Shutting down camera..." << std::endl;
                    camera.Close();
                    std::cout << "Camera successfully closed." << std::endl;
                    break;
                }

                ////////////////////////////////////////////////////
                //////////// Here your code ends ///////////////////
                ////////////////////////////////////////////////////




                frame++;

            }
            else
            {
                std::cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << std::endl;
            }
        }

    }
    catch (GenICam::GenericException &e)
    {
        // Error handling.
        std::cerr << "An exception occurred." << std::endl
        << e.GetDescription() << std::endl;
        exitCode = 1;
    }

    return exitCode;
}

std::pair<cv::Mat,cv::Mat> findCalibrationVariables(std::string path) {
    std::vector<cv::String> fileNames;
    cv::glob(path, fileNames, false);
    cv::Size patternSize(7 - 1, 10  - 1);
    std::vector<std::vector<cv::Point2f>> q(fileNames.size());

    // Detect feature points
    std::size_t i = 0;
    for (auto const &f : fileNames) {
      std::cout << std::string(f) << std::endl;

      cv::Mat img = cv::imread(f);
      bool success;
      // 1. Read in the image an call cv::findChessboardCorners()
      success = cv::findChessboardCorners(img, patternSize, q[i]);
  //    std::cout << success << std::endl;

      // 2. Use cv::cornerSubPix() to refine the found corner detections
      cv::Mat grey(img.rows, img.rows, img.type(), cv::Scalar(0,0,0));
      cv::cvtColor(img, grey, cv::COLOR_BGR2GRAY);
      if (success) {
          cv::cornerSubPix(grey, q[i], cv::Size(4,4), cv::Size(-1,-1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.01));
      }
      // Display
      cv::drawChessboardCorners(grey, patternSize, q[i], success);

      i++;
    }

    std::vector<cv::Point3f> QView;
    for (int i = 0; i < patternSize.width; ++i) {
        for (int j = 0; j < patternSize.height; ++j) {
            cv::Point3f p3f(static_cast<float>(j)*51,static_cast<float>(i)*51, 0.f);
            QView.push_back(p3f);
        }
    }

    std::vector<std::vector<cv::Point3f>> Q(fileNames.size());
    for (int i = 0; i < fileNames.size(); ++i) { Q[i] = QView; }

    cv::Matx33f K(cv::Matx33f::eye());  // intrinsic camera matrix
    cv::Vec<float, 5> k(0, 0, 0, 0, 0); // distortion coefficients

    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<double> stdIntrinsics, stdExtrinsics, perViewErrors;
    int flags = cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_K3 +
                cv::CALIB_ZERO_TANGENT_DIST + cv::CALIB_FIX_PRINCIPAL_POINT;
    cv::Size frameSize(1440, 1080);

    std::cout << "Calibrating..." << std::endl;
    // 4. Call "float error = cv::calibrateCamera()" with the input coordinates
    // and output parameters as declared above...

    float error = cv::calibrateCamera(Q, q, frameSize, K, k, rvecs, tvecs, stdIntrinsics, stdExtrinsics, perViewErrors, flags);

    std::cout << "Reprojection error = " << error << "\nK =\n"
              << K << "\nk=\n"
              << k << std::endl;

    // Precompute lens correction interpolation
    cv::Mat mapX, mapY;
    cv::initUndistortRectifyMap(K, k, cv::Matx33f::eye(), K, frameSize, CV_32FC1,
                                mapX, mapY);
    std::pair<cv::Mat,cv::Mat> calibrationVariables(mapX, mapY);
    return calibrationVariables;
}
