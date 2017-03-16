#include "GripPipeline.h"
/**
* Initializes a GripPipeline.
*/

namespace grip {

	GripPipeline::GripPipeline()
	{
	}

	/**
	 * Runs an iteration of the Pipeline and updates outputs.
	 *
	 * Sources need to be set before calling this method.
	 *
	 */
	void
	GripPipeline::process(cv::Mat source0)
	{
		// Step RGB_Threshold0:
		cv::Mat rgbThresholdInput = source0;
		double rgbThresholdRed[] = {0.0, 255.0};
		double rgbThresholdGreen[] = {130, 255.0};
		double rgbThresholdBlue[] = {0.0, 255.0};
		rgbThreshold(rgbThresholdInput, rgbThresholdRed, rgbThresholdGreen, rgbThresholdBlue, this->rgbThresholdOutput);
		// Step Find_Contours0:
		cv::Mat findContoursInput = rgbThresholdOutput;
		bool findContoursExternalOnly = false;  // default Boolean
		findContours(findContoursInput, findContoursExternalOnly, this->findContoursOutput);
	}

	/**
	 * This method is a generated setter for source0.
	 * @param source the Mat to set
	 */
	void
	GripPipeline::setsource0(cv::Mat &source0)
	{
		source0.copyTo(this->source0);
	}

	/**
	 * This method is a generated getter for the output of a RGB_Threshold.
	 * @return Mat output from RGB_Threshold.
	 */
	cv::Mat*
	GripPipeline::getrgbThresholdOutput()
	{
		return &(this->rgbThresholdOutput);
	}

	/**
	 * This method is a generated getter for the output of a Find_Contours.
	 * @return ContoursReport output from Find_Contours.
	 */
	std::vector<std::vector<cv::Point> >*
	GripPipeline::getfindContoursOutput()
	{
		return &(this->findContoursOutput);
	}

	/**
	 * Segment an image based on color ranges.
	 *
	 * @param input The image on which to perform the RGB threshold.
	 * @param red The min and max red.
	 * @param green The min and max green.
	 * @param blue The min and max blue.
	 * @param output The image in which to store the output.
	 */
	void
	GripPipeline::rgbThreshold(cv::Mat &input,
		double red[], double green[], double blue[], cv::Mat &output)
	{
		cv::cvtColor(input, output, cv::COLOR_BGR2RGB);
		cv::inRange(output, cv::Scalar(red[0], green[0], blue[0]), cv::Scalar(red[1], green[1], blue[1]), output);
	}

	/**
	 * Finds contours in an image.
	 *
	 * @param input The image to find contours in.
	 * @param externalOnly if only external contours are to be found.
	 * @param contours vector of contours to put contours in.
	 */
	void
	GripPipeline::findContours(cv::Mat &input, bool externalOnly,
			std::vector<std::vector<cv::Point> > &contours)
	{
		std::vector<cv::Vec4i> hierarchy;
		contours.clear();
		int mode = externalOnly ? cv::RETR_EXTERNAL : cv::RETR_LIST;
		int method = cv::CHAIN_APPROX_SIMPLE;
		cv::findContours(input, contours, hierarchy, mode, method);
	}

} // end grip namespace

