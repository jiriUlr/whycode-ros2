#ifndef WHYCODE__CWHYCON_H
#define WHYCODE__CWHYCON_H

#include <vector>
#include <deque>
#include <map>
#include <array>
#include <stdlib.h>
#include <string>
#include <cmath>
#include <memory>

// WhyCode libs
#include "whycode/CCircleDetect.h"
#include "whycode/CTransformation.h"
#include "whycode/CNecklace.h"
#include "whycode/CRawImage.h"

namespace whycode
{

class CWhycode {

    public:

        float circle_diameter_;         // default black circle diameter [m];
        float field_length_ = 1.0;      // X dimension of the coordinate system
        float field_width_ = 1.0;       // Y dimension of the coordinate system

        // marker detection variables
        bool identify_ = false;     // whether to identify ID
        int num_markers_ = 0;           // num of markers to track
        int num_found_ = 0;         // num of markers detected in the last step
        int num_static_ = 0;        // num of non-moving markers

        // marker identification
        int id_bits_;       // num of ID bits
        int id_samples_;    // num of samples to identify ID
        int hamming_dist_;  // hamming distance of ID code

        ~CWhycode();
        
        void init(float circle_diam, int id_b, int id_s, int ham_dist);

        void setDrawing(bool draw_coords, bool draw_segments);

        void setCoordinates(ETransformType type);

        void autocalibration();

        void manualcalibration();

        void selectMarker(float x, float y);

        void updateCameraInfo(const std::array<double, 9> &intrinsic_mat, const std::vector<double> &distortion_coeffs);

        void processImage(CRawImage &image, std::vector<SMarker> &whycode_detections);

        bool getDrawCoords();

        bool getDrawSegments();

        int getCoordinates();

        void set_parameters(Parameters &params);

        Parameters get_parameters();

        void setCalibrationConfig(const CalibrationConfig &config);

        CalibrationConfig getCalibrationConfig();

    private:

        Parameters params_;

        // GUI-related stuff
        bool use_gui_;          // whether use graphic interface
        bool draw_coords_ = true;      // draw coordinates at the marker's positions
        bool draw_segments_ = false;    // draw segmentation outcome
        int eval_time_;         // time required to detect the patterns

        CTransformation *trans_;    // transformation instance
        CNecklace *decoder_;        // instance to decode marker's ID

        std::vector<SMarker> current_marker_array_;      // array of currently detected markers
        std::vector<SMarker> last_marker_array_;         // array of previously detected markers
        std::vector<std::unique_ptr<CCircleDetect>> detector_array_;     // array of detector instances for each marker

        bool calibrated_coords_ = false;


        std::array<std::map<int, int>, 4> indices_autocalib;
        int index_autocalib[4];

        // variables related to (auto) calibration
        const int calibration_steps_ = 20;              // how many measurements to average to estimate calibration pattern position (manual calib)
        const int auto_calibration_steps_ = 30;         // how many measurements to average to estimate calibration pattern position (automatic calib)  
        const int auto_calibration_pre_steps_ = 10;     // how many measurements to discard before starting to actually auto-calibrating (automatic calib)  
        int calib_num_ = 5;                             // number of objects acquired for calibration (5 means calibration winished inactive)
        STrackedObject calib_[4];                       // array to store calibration patterns positions
        std::vector<STrackedObject> calib_tmp_;         // array to store several measurements of a given calibration pattern
        int calib_step_ = calibration_steps_ + 2;       // actual calibration step (num of measurements of the actual pattern)
        
        bool autocalibrate_ = false;                    // is the autocalibration in progress ?
        bool mancalibrate_ = false;

        ETransformType last_transform_type_ = TRANSFORM_2D;     // pre-calibration transform (used to preserve pre-calibation transform type)
        int was_markers_ = 1;                                   // pre-calibration number of makrers to track (used to preserve pre-calibation number of markers to track)

        
        

        /*manual calibration can be initiated by pressing 'r' and then clicking circles at four positions (0,0)(fieldLength,0)...*/
        void manualCalib();

        /*finds four outermost circles and uses them to set-up the coordinate system - [0,0] is left-top, [0,fieldLength] next in clockwise direction*/
        void autoCalib();
};

}

#endif  // WHYCODE__CWHYCON_H
