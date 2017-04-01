//colour detection for detecting stop sign
	Mat HSVMat;
	Mat maskMat;
	Scalar lowerRed = (160,100,100);
	Scalar upperRed = (179,255,255);
	cvtColor(capMat,HSVMat,CV_BGR2HSV);
	inRange(HSVMat,lowerRed,upperRed,maskMat);
	imshow("colour threshold mask",maskMat);