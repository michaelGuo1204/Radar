
#include "mapwork/Gp.hpp"


Gp::Gp(){
	
	
}


Node0 *Gp::Create_Node(double info1, double info2, double info3)
{
	N_ptr = new Node0;
	if (N_ptr != NULL)
	{
		N_ptr->Next = NULL;
		N_ptr->no_of_components = 1;
		N_ptr->pixel_s = N_ptr->pixel_r = Create_gaussian(info1, info2, info3);
	}
	return N_ptr;
}

gaussian *Gp::Create_gaussian(double info1, double info2, double info3)
{
	ptr = new gaussian;
	if (ptr != NULL)
	{
		ptr->mean[0] = info1;
		ptr->mean[1] = info2;
		ptr->mean[2] = info3;
		ptr->covariance = covariance0;
		ptr->weight = alpha;
		ptr->Next = NULL;
		ptr->Previous = NULL;
	}
	return ptr;
}

void Gp::Insert_End_Node(Node0 *np)
{
	if (N_start != NULL)
	{
		N_rear->Next = np;
		N_rear = np;
	}
	else
		N_start = N_rear = np;
}

void Gp::Insert_End_gaussian(gaussian *nptr)
{
	if (start != NULL)
	{
		rear->Next = nptr;
		nptr->Previous = rear;
		rear = nptr;
	}
	else
		start = rear = nptr;
}

gaussian *Gp::Delete_gaussian(gaussian *nptr)
{
	previous = nptr->Previous;
	next = nptr->Next;
	if (start != NULL)
	{
		if (nptr == start && nptr == rear)
		{
			start = rear = NULL;
			delete nptr;
		}
		else if (nptr == start)
		{
			next->Previous = NULL;
			start = next;
			delete nptr;
			nptr = start;
		}
		else if (nptr == rear)
		{
			previous->Next = NULL;
			rear = previous;
			delete nptr;
			nptr = rear;
		}
		else
		{
			previous->Next = next;
			next->Previous = previous;
			delete nptr;
			nptr = next;
		}
	}
	else
	{
		std::cout << "Underflow........";
		exit(0);
	}
	return nptr;
}

int Gp::Gpmain(cv::Mat frame,cv::Mat &gp){
	
	waitKey(10);
	if(first_time){
		i = j = k = 0;

		// Declare matrices to store original and resultant binary image
		
		
		cv::cvtColor(frame, frame, CV_BGR2GRAY);

		//Initializing the binary image with the same dimensions as original image
		bin_img = cv::Mat(frame.cols,frame.rows,  CV_8U, cv::Scalar(0));
		std::cout << "2" <<std::endl;
		//Step 1: initializing with one gaussian for the first time and keeping the no. of models as 1
		
		for (i = 0; i < frame.rows; i++)
		{
			r_ptr = frame.ptr(i);
			for (j = 0; j < frame.cols; j++)
			{

				N_ptr = Create_Node(*r_ptr, *(r_ptr + 1), *(r_ptr + 2));
				if (N_ptr != NULL)
				{
					N_ptr->pixel_s->weight = 1.0;
					Insert_End_Node(N_ptr);
				}
				else
				{
					std::cout << "Memory limit reached... ";
					exit(0);
				}
			}
		}

		// cap >> frame;
		if (frame.empty())
		{
			std::cout << "empty img!" << std::endl;
		}
		
		if (frame.isContinuous() == true)
		{
			nL = 1;
			nC = frame.rows * frame.cols * frame.channels();
		}
		else
		{
			nL = frame.rows;
			nC = frame.cols * frame.channels();
		}
		first_time = false;
	}
	//std::cout << "3" <<std::endl;
	//Step 2: Modelling each pixel with Gaussian
	duration1 = static_cast<double>(cv::getTickCount());
	bin_img = cv::Mat(frame.rows, frame.cols, CV_8UC1, cv::Scalar(0));
	cv::imshow("bin",bin_img);
	duration3 = 0.0;
	// // cap >> frame;
	// if (frame.empty())
	// {
	// 	std::cout << "empty img!" << std::endl;
	// 	return 1;
	// }
	int count = 0;
	N_ptr = N_start;
	duration = static_cast<double>(cv::getTickCount());
	for (i = 0; i < nL; i++)
	{
		r_ptr = frame.ptr(i);
		b_ptr = bin_img.ptr(i);
		for (j = 0; j < nC; j += 3)
		{
			sum = 0.0;
			close = false;
			background = 0;

			rVal = *(r_ptr++);
			gVal = *(r_ptr++);
			bVal = *(r_ptr++);

			start = N_ptr->pixel_s;
			rear = N_ptr->pixel_r;
			ptr = start;

			temp_ptr = NULL;

			if (N_ptr->no_of_components > 4)
			{
				Delete_gaussian(rear);
				N_ptr->no_of_components--;
			}

			for (k = 0; k < N_ptr->no_of_components; k++)
			{

				weight = ptr->weight;
				mult = alpha / weight;
				weight = weight * alpha_bar + prune;
				if (close == false)
				{
					muR = ptr->mean[0];
					muG = ptr->mean[1];
					muB = ptr->mean[2];

					dR = rVal - muR;
					dG = gVal - muG;
					dB = bVal - muB;
					var = ptr->covariance;

					mal_dist = (dR * dR + dG * dG + dB * dB);

					if ((sum < 0.5 * cfbar) && (mal_dist < 5.0 * var * var))
						background = 255;

					if (mal_dist < 5.0 * var * var)
					{
						weight += alpha;
						close = true;
						ptr->mean[0] = muR + mult * dR;
						ptr->mean[1] = muG + mult * dG;
						ptr->mean[2] = muB + mult * dB;
						temp_cov = var + mult * (mal_dist - var);
						ptr->covariance = temp_cov < 5.0 ? 5.0 : (temp_cov > 20.0 ? 20.0 : temp_cov);
						temp_ptr = ptr;
					}
				}

				if (weight < -prune)
				{
					ptr = Delete_gaussian(ptr);
					weight = 0;
					N_ptr->no_of_components--;
				}
				else
				{
					sum += weight;
					ptr->weight = weight;
				}

				ptr = ptr->Next;
			}

			if (close == false)
			{
				ptr = new gaussian;
				ptr->weight = alpha;
				ptr->mean[0] = rVal;
				ptr->mean[1] = gVal;
				ptr->mean[2] = bVal;
				ptr->covariance = covariance0;
				ptr->Next = NULL;
				ptr->Previous = NULL;
				if (start == NULL)
					start = rear = NULL;
				else
				{
					ptr->Previous = rear;
					rear->Next = ptr;
					rear = ptr;
				}
				temp_ptr = ptr;
				N_ptr->no_of_components++;
			}

			ptr = start;
			while (ptr != NULL)
			{
				ptr->weight /= sum;
				ptr = ptr->Next;
			}

			while (temp_ptr != NULL && temp_ptr->Previous != NULL)
			{
				if (temp_ptr->weight <= temp_ptr->Previous->weight)
					break;
				else
				{
					next = temp_ptr->Next;
					previous = temp_ptr->Previous;
					if (start == previous)
						start = temp_ptr;
					previous->Next = next;
					temp_ptr->Previous = previous->Previous;
					temp_ptr->Next = previous;
					if (previous->Previous != NULL)
						previous->Previous->Next = temp_ptr;
					if (next != NULL)
						next->Previous = previous;
					else
						rear = previous;
					previous->Previous = temp_ptr;
				}

				temp_ptr = temp_ptr->Previous;
			}
			N_ptr->pixel_s = start;
			N_ptr->pixel_r = rear;
			*b_ptr++ = background;
			N_ptr = N_ptr->Next;
		}
	}
	//std::cout << "4" <<std::endl;
	duration = static_cast<double>(cv::getTickCount()) - duration;
	duration /= cv::getTickFrequency();

	std::cout << "\n duration :" << duration;
	std::cout << "\n counts : " << count;
	//cv::imshow("video", frame);
	//cv::imshow("gp1", bin_img);
	cv::Mat kernel_e = getStructuringElement(cv::MORPH_RECT,Size(7,7),Point(3,3));
	cv::Mat kernel_d = getStructuringElement(cv::MORPH_RECT,Size(3,3),Point(1,1));
	cv::dilate(bin_img,bin_img,kernel_d);
	cv::erode(bin_img,bin_img,kernel_e);
	//cv::imshow("gp", bin_img);
	gp = bin_img.clone();
		

	duration1 = static_cast<double>(cv::getTickCount()) - duration1;
	duration1 /= cv::getTickFrequency();

	std::cout << "\n duration1 :" << duration1;

	return 0;
}
