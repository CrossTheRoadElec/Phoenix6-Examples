#pragma once

///
//https://github.com/LimelightVision/limelightlib-wpicpp
///

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include <wpinet/PortForwarder.h>
#include "wpi/json.h"
#include <chrono>
#include <iostream>
#include <optional>
#include <string>
#include <vector>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Rotation3d.h>
//#include <unistd.h>
//#include <curl/curl.h>
// #include <sys/socket.h>
// #include <netinet/in.h>
// #include <arpa/inet.h>
#include <cstring>
#include <fcntl.h>
    
namespace LimelightHelpers
{
    inline std::string sanitizeName(const std::string &name)
    {
        if (name == "")
        {
            return "limelight";
        }
        return name;
    }

    inline frc::Pose3d toPose3D(const std::vector<double>& inData)
    {
        if(inData.size() < 6)
        {
            return frc::Pose3d();
        }
        return frc::Pose3d(
            frc::Translation3d(units::length::meter_t(inData[0]), units::length::meter_t(inData[1]), units::length::meter_t(inData[2])),
            frc::Rotation3d(units::angle::degree_t(inData[3]), units::angle::degree_t(inData[4]),
                   units::angle::degree_t(inData[5])));
    }

    inline frc::Pose2d toPose2D(const std::vector<double>& inData)
    {
        if(inData.size() < 6)
        {
            return frc::Pose2d();
        }
        return frc::Pose2d(
            frc::Translation2d(units::length::meter_t(inData[0]), units::length::meter_t(inData[1])), 
            frc::Rotation2d(units::angle::degree_t(inData[5])));
    }

    inline std::shared_ptr<nt::NetworkTable> getLimelightNTTable(const std::string &tableName)
    {
        return nt::NetworkTableInstance::GetDefault().GetTable(sanitizeName(tableName));
    }

    inline nt::NetworkTableEntry getLimelightNTTableEntry(const std::string &tableName, const std::string &entryName)
    {
        return getLimelightNTTable(tableName)->GetEntry(entryName);
    }

    inline double getLimelightNTDouble(const std::string &tableName, const std::string &entryName)
    {
        return getLimelightNTTableEntry(tableName, entryName).GetDouble(0.0);
    }

    inline std::vector<double> getLimelightNTDoubleArray(const std::string &tableName, const std::string &entryName)
    {
        return getLimelightNTTableEntry(tableName, entryName).GetDoubleArray(std::span<double>{});
    }

    inline std::string getLimelightNTString(const std::string &tableName, const std::string &entryName)
    {
        return getLimelightNTTableEntry(tableName, entryName).GetString("");
    }

    inline void setLimelightNTDouble(const std::string &tableName, const std::string entryName, double val)
    {
        getLimelightNTTableEntry(tableName, entryName).SetDouble(val);
    }

    inline void setLimelightNTDoubleArray(const std::string &tableName, const std::string &entryName, const std::span<const double> &vals)
    {
        getLimelightNTTableEntry(tableName, entryName).SetDoubleArray(vals);
    }

    inline double getTX(const std::string &limelightName = "")
    {
        return getLimelightNTDouble(limelightName, "tx");
    }
    
    inline double getTV(const std::string &limelightName = "")
    {
        return getLimelightNTDouble(limelightName, "tv");
    }

    inline double getTY(const std::string &limelightName = "")
    {
        return getLimelightNTDouble(limelightName, "ty");
    }

    inline double getTA(const std::string &limelightName = "")
    {
        return getLimelightNTDouble(limelightName, "ta");
    }

    inline double getLatency_Pipeline(const std::string &limelightName = "")
    {
        return getLimelightNTDouble(limelightName, "tl");
    }

    inline double getLatency_Capture(const std::string &limelightName = "")
    {
        return getLimelightNTDouble(limelightName, "cl");
    }

    inline std::string getJSONDump(const std::string &limelightName = "")
    {
        return getLimelightNTString(limelightName, "json");
    }

    inline std::vector<double> getBotpose(const std::string &limelightName = "")
    {
        return getLimelightNTDoubleArray(limelightName, "botpose");
    }

    inline std::vector<double> getBotpose_wpiRed(const std::string &limelightName = "")
    {
        return getLimelightNTDoubleArray(limelightName, "botpose_wpired");
    }

    inline std::vector<double> getBotpose_wpiBlue(const std::string &limelightName = "")
    {
        return getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
    }



    inline std::vector<double> getBotpose_TargetSpace(const std::string &limelightName = "")
    {
        return getLimelightNTDoubleArray(limelightName, "botpose_targetspace");
    }

    inline std::vector<double> getCameraPose_TargetSpace(const std::string &limelightName = "")
    {
        return getLimelightNTDoubleArray(limelightName, "camerapose_targetspace");
    }

    inline std::vector<double> getCameraPose_RobotSpace(const std::string &limelightName = "")
    {
        return getLimelightNTDoubleArray(limelightName, "camerapose_robotspace");
    }

    inline std::vector<double> getTargetPose_CameraSpace(const std::string &limelightName = "")
    {
        return getLimelightNTDoubleArray(limelightName, "targetpose_cameraspace");
    }

    inline std::vector<double> getTargetPose_RobotSpace(const std::string &limelightName = "")
    {
        return getLimelightNTDoubleArray(limelightName, "targetpose_robotspace");
    }

    inline std::vector<double> getTargetColor(const std::string &limelightName = "")
    {
        return getLimelightNTDoubleArray(limelightName, "tc");
    }

    inline double getFiducialID(const std::string &limelightName = "")
    {
        return getLimelightNTDouble(limelightName, "tid");
    }

    inline std::string getNeuralClassID(const std::string &limelightName = "")
    {
        return getLimelightNTString(limelightName, "tclass");
    }

    /////
    /////

    inline void setPipelineIndex(const std::string &limelightName, int index)
    {
        setLimelightNTDouble(limelightName, "pipeline", index);
    }

    inline void setPriorityTagID(const std::string &limelightName, int ID) {
        setLimelightNTDouble(limelightName, "priorityid", ID);
    }

    inline void setLEDMode_PipelineControl(const std::string &limelightName = "")
    {
        setLimelightNTDouble(limelightName, "ledMode", 0);
    }

    inline void setLEDMode_ForceOff(const std::string &limelightName = "")
    {
        setLimelightNTDouble(limelightName, "ledMode", 1);
    }

    inline void setLEDMode_ForceBlink(const std::string &limelightName = "")
    {
        setLimelightNTDouble(limelightName, "ledMode", 2);
    }

    inline void setLEDMode_ForceOn(const std::string &limelightName = "")
    {
        setLimelightNTDouble(limelightName, "ledMode", 3);
    }

    inline void setStreamMode_Standard(const std::string &limelightName = "")
    {
        setLimelightNTDouble(limelightName, "stream", 0);
    }

    inline void setStreamMode_PiPMain(const std::string &limelightName = "")
    {
        setLimelightNTDouble(limelightName, "stream", 1);
    }

    inline void setStreamMode_PiPSecondary(const std::string &limelightName = "")
    {
        setLimelightNTDouble(limelightName, "stream", 2);
    }

    /**
     * Sets the crop window. The crop window in the UI must be completely open for
     * dynamic cropping to work.
     */
    inline void setCropWindow(const std::string &limelightName, double cropXMin,
                              double cropXMax, double cropYMin, double cropYMax)
    {
        double cropWindow[4]{cropXMin, cropXMax, cropYMin, cropYMax};
        setLimelightNTDoubleArray(limelightName, "crop", cropWindow);
    }

    /**
     * Sets the robot orientation for mt2.
     */
    inline void SetRobotOrientation(const std::string& limelightName, 
        double yaw, double yawRate, 
        double pitch, double pitchRate, 
        double roll, double rollRate) 
    {
        std::vector<double> entries = {yaw, yawRate, pitch, pitchRate, roll, rollRate};
        setLimelightNTDoubleArray(limelightName, "robot_orientation_set", entries);
    }

    inline void SetFiducialDownscaling(const std::string& limelightName, float downscale) 
    {
        int d = 0; // pipeline
        if (downscale == 1.0)
        {
            d = 1;
        }
        if (downscale == 1.5)
        {
            d = 2;
        }
        if (downscale == 2)
        {
            d = 3;
        }
        if (downscale == 3)
        {
            d = 4;
        }
        if (downscale == 4)
        {
            d = 5;
        }
        setLimelightNTDouble(limelightName, "fiducial_downscale_set", d);
    }

    inline void SetFiducialIDFiltersOverride(const std::string& limelightName, const std::vector<int>& validIDs) 
    {
        std::vector<double> validIDsDouble(validIDs.begin(), validIDs.end());
        setLimelightNTDoubleArray(limelightName, "fiducial_id_filters_set", validIDsDouble);
    }

    /////
    /////

    /**
     * Sets the camera pose in robotspace. The UI camera pose must be set to zeros
     */
    inline void setCameraPose_RobotSpace(const std::string &limelightName, double forward, double side, double up, double roll, double pitch, double yaw) {
        double entries[6] ={forward, side, up, roll, pitch, yaw};
        setLimelightNTDoubleArray(limelightName, "camerapose_robotspace_set", entries);
    }

    inline void setPythonScriptData(const std::string &limelightName, const std::vector<double> &outgoingPythonData)
    {
        setLimelightNTDoubleArray(limelightName, "llrobot", std::span{outgoingPythonData.begin(), outgoingPythonData.size()});
    }

    inline std::vector<double> getPythonScriptData(const std::string &limelightName = "")
    {
        return getLimelightNTDoubleArray(limelightName, "llpython");
    }


    inline double extractArrayEntry(const std::vector<double>& inData, int position) {
        if (inData.size() < static_cast<size_t>(position + 1)) {
            return 0.0;
        }
        return inData[position];
    }

    class RawFiducial 
    {
    public:
        int id{0};
        double txnc{0.0};
        double tync{0.0};
        double ta{0.0};
        double distToCamera{0.0};
        double distToRobot{0.0};
        double ambiguity{0.0};

        RawFiducial(int id, double txnc, double tync, double ta, double distToCamera, double distToRobot, double ambiguity)
            : id(id), txnc(txnc), tync(tync), ta(ta), distToCamera(distToCamera), distToRobot(distToRobot), ambiguity(ambiguity) {}
    };

    inline std::vector<RawFiducial> getRawFiducials(const std::string& limelightName) 
    {
        nt::NetworkTableEntry entry = LimelightHelpers::getLimelightNTTableEntry(limelightName, "rawfiducials");
        std::vector<double> rawFiducialArray = entry.GetDoubleArray({});
        int valsPerEntry = 7;
        if (rawFiducialArray.size() % valsPerEntry != 0) {
            return {};
        }

        int numFiducials = rawFiducialArray.size() / valsPerEntry;
        std::vector<RawFiducial> rawFiducials;

        for (int i = 0; i < numFiducials; ++i) {
            int baseIndex = i * valsPerEntry;
            int id = static_cast<int>(extractArrayEntry(rawFiducialArray, baseIndex));
            double txnc = extractArrayEntry(rawFiducialArray, baseIndex + 1);
            double tync = extractArrayEntry(rawFiducialArray, baseIndex + 2);
            double ta = extractArrayEntry(rawFiducialArray, baseIndex + 3);
            double distToCamera = extractArrayEntry(rawFiducialArray, baseIndex + 4);
            double distToRobot = extractArrayEntry(rawFiducialArray, baseIndex + 5);
            double ambiguity = extractArrayEntry(rawFiducialArray, baseIndex + 6);

            rawFiducials.emplace_back(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity);
        }

        return rawFiducials;
    }


    class RawDetection 
    {
    public:
        int classId{-1};
        double txnc{0.0};
        double tync{9.0}; // It seems like you intentionally set this to 9.0, so I kept it as is.
        double ta{0.0};
        double corner0_X{0.0};
        double corner0_Y{0.0};
        double corner1_X{0.0};
        double corner1_Y{0.0};
        double corner2_X{0.0};
        double corner2_Y{0.0};
        double corner3_X{0.0};
        double corner3_Y{0.0};

        RawDetection(int classId, double txnc, double tync, double ta, 
                    double corner0_X, double corner0_Y, 
                    double corner1_X, double corner1_Y, 
                    double corner2_X, double corner2_Y, 
                    double corner3_X, double corner3_Y)
            : classId(classId), txnc(txnc), tync(tync), ta(ta), 
            corner0_X(corner0_X), corner0_Y(corner0_Y), 
            corner1_X(corner1_X), corner1_Y(corner1_Y), 
            corner2_X(corner2_X), corner2_Y(corner2_Y), 
            corner3_X(corner3_X), corner3_Y(corner3_Y) {}
    };

    inline std::vector<RawDetection> getRawDetections(const std::string& limelightName) 
    {
        nt::NetworkTableEntry entry = LimelightHelpers::getLimelightNTTableEntry(limelightName, "rawdetections");
        std::vector<double> rawDetectionArray = entry.GetDoubleArray({});
        int valsPerEntry = 11;

        if (rawDetectionArray.size() % valsPerEntry != 0) {
            return {};
        }

        int numDetections = rawDetectionArray.size() / valsPerEntry;
        std::vector<RawDetection> rawDetections;

        for (int i = 0; i < numDetections; ++i) {
            int baseIndex = i * valsPerEntry;
            int classId = static_cast<int>(extractArrayEntry(rawDetectionArray, baseIndex));
            double txnc = extractArrayEntry(rawDetectionArray, baseIndex + 1);
            double tync = extractArrayEntry(rawDetectionArray, baseIndex + 2);
            double ta = extractArrayEntry(rawDetectionArray, baseIndex + 3);
            double corner0_X = extractArrayEntry(rawDetectionArray, baseIndex + 4);
            double corner0_Y = extractArrayEntry(rawDetectionArray, baseIndex + 5);
            double corner1_X = extractArrayEntry(rawDetectionArray, baseIndex + 6);
            double corner1_Y = extractArrayEntry(rawDetectionArray, baseIndex + 7);
            double corner2_X = extractArrayEntry(rawDetectionArray, baseIndex + 8);
            double corner2_Y = extractArrayEntry(rawDetectionArray, baseIndex + 9);
            double corner3_X = extractArrayEntry(rawDetectionArray, baseIndex + 10);
            double corner3_Y = extractArrayEntry(rawDetectionArray, baseIndex + 11);

            rawDetections.emplace_back(classId, txnc, tync, ta, corner0_X, corner0_Y, corner1_X, corner1_Y, corner2_X, corner2_Y, corner3_X, corner3_Y);
        }

        return rawDetections;
    }

    class PoseEstimate
    {
    public:
        frc::Pose2d pose;
        units::time::second_t timestampSeconds{0.0};
        double latency{0.0};
        int tagCount{0};
        double tagSpan{0.0};
        double avgTagDist{0.0};
        double avgTagArea{0.0};
        std::vector<RawFiducial> rawFiducials;

        PoseEstimate() = default;

         PoseEstimate(const frc::Pose2d& pose, units::time::second_t timestampSeconds, 
            double latency, int tagCount, double tagSpan, double avgTagDist, double avgTagArea,
            const std::vector<RawFiducial>& rawFiducials)
            : pose(pose), timestampSeconds(timestampSeconds), 
                latency(latency), tagCount(tagCount), tagSpan(tagSpan), 
                avgTagDist(avgTagDist), avgTagArea(avgTagArea), rawFiducials(rawFiducials)
        {
        }
    };

    inline std::optional<PoseEstimate> getBotPoseEstimate(const std::string& limelightName, const std::string& entryName) {
        nt::NetworkTableEntry poseEntry = getLimelightNTTableEntry(limelightName, entryName);
        std::vector<double> poseArray = poseEntry.GetDoubleArray(std::span<double>{});
        frc::Pose2d pose = toPose2D(poseArray);

        if (poseArray.size() == 0) {
            // Handle the case where no data is available
            return std::nullopt; // or some default PoseEstimate
        }

        double latency = extractArrayEntry(poseArray, 6);
        int tagCount = static_cast<int>(extractArrayEntry(poseArray, 7));
        double tagSpan = extractArrayEntry(poseArray, 8);
        double tagDist = extractArrayEntry(poseArray, 9);
        double tagArea = extractArrayEntry(poseArray, 10);

        // getLastChange: microseconds; latency: milliseconds
        units::time::second_t timestamp = units::time::second_t((poseEntry.GetLastChange() / 1000000.0) - (latency / 1000.0));

        std::vector<RawFiducial> rawFiducials;
        constexpr int valsPerFiducial = 7;
        size_t expectedTotalVals = 11 + valsPerFiducial * tagCount;
        
        if (poseArray.size() == expectedTotalVals) 
        {
            for (int i = 0; i < tagCount; i++) 
            {
                int baseIndex = 11 + (i * valsPerFiducial);
                int id = static_cast<int>(extractArrayEntry(poseArray, baseIndex));
                double txnc = extractArrayEntry(poseArray, baseIndex + 1);
                double tync = extractArrayEntry(poseArray, baseIndex + 2);
                double ta = extractArrayEntry(poseArray, baseIndex + 3);
                double distToCamera = extractArrayEntry(poseArray, baseIndex + 4);
                double distToRobot = extractArrayEntry(poseArray, baseIndex + 5);
                double ambiguity = extractArrayEntry(poseArray, baseIndex + 6);
                rawFiducials.emplace_back(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity);
            }
        }

        return PoseEstimate(pose, timestamp, latency, tagCount, tagSpan, tagDist, tagArea, rawFiducials);
    }

    inline std::optional<PoseEstimate> getBotPoseEstimate_wpiBlue(const std::string &limelightName = "") {
        return getBotPoseEstimate(limelightName, "botpose_wpiblue");
    }

    inline std::optional<PoseEstimate> getBotPoseEstimate_wpiRed(const std::string &limelightName = "") {
        return getBotPoseEstimate(limelightName, "botpose_wpired");
    }

    inline std::optional<PoseEstimate> getBotPoseEstimate_wpiBlue_MegaTag2(const std::string &limelightName = "") {
        return getBotPoseEstimate(limelightName, "botpose_orb_wpiblue");
    }

    inline std::optional<PoseEstimate> getBotPoseEstimate_wpiRed_MegaTag2(const std::string &limelightName = "") {
        return getBotPoseEstimate(limelightName, "botpose_orb_wpired");
    }
     
    inline const double INVALID_TARGET = 0.0;
    class SingleTargetingResultClass
    {
    public:
        SingleTargetingResultClass(){};
        ~SingleTargetingResultClass(){};
        double m_TargetXPixels{INVALID_TARGET};
        double m_TargetYPixels{INVALID_TARGET};

        double m_TargetXNormalized{INVALID_TARGET};
        double m_TargetYNormalized{INVALID_TARGET};

        double m_TargetXNormalizedCrosshairAdjusted{INVALID_TARGET};
        double m_TargetYNormalizedCrosshairAdjusted{INVALID_TARGET};

        double m_TargetXDegreesCrosshairAdjusted{INVALID_TARGET};
        double m_TargetYDegreesCrosshairAdjusted{INVALID_TARGET};

        double m_TargetAreaPixels{INVALID_TARGET};
        double m_TargetAreaNormalized{INVALID_TARGET};
        double m_TargetAreaNormalizedPercentage{INVALID_TARGET};

        // not included in json//
        double m_timeStamp{-1.0};
        double m_latency{0};
        double m_pipelineIndex{-1.0};
        std::vector<std::vector<double>> m_TargetCorners;

        std::vector<double> m_CAMERATransform6DTARGETSPACE;
        std::vector<double> m_TargetTransform6DCAMERASPACE;
        std::vector<double> m_TargetTransform6DROBOTSPACE;
        std::vector<double> m_ROBOTTransform6DTARGETSPACE;
        std::vector<double> m_ROBOTTransform6DFIELDSPACE;
        std::vector<double> m_CAMERATransform6DROBOTSPACE;

    };

    class RetroreflectiveResultClass : public SingleTargetingResultClass
    {
    public:
        RetroreflectiveResultClass() {}
        ~RetroreflectiveResultClass() {}
    };

    class FiducialResultClass : public SingleTargetingResultClass
    {
    public:
        FiducialResultClass() {}
        ~FiducialResultClass() {}
        int m_fiducialID{0};
        std::string m_family{""};
    };

    class DetectionResultClass : public SingleTargetingResultClass
    {
    public:
        DetectionResultClass() {}
        ~DetectionResultClass() {}

        int m_classID{-1};
        std::string m_className{""};
        double m_confidence{0};
    };

    class ClassificationResultClass : public SingleTargetingResultClass
    {
    public:
        ClassificationResultClass() {}
        ~ClassificationResultClass() {}

        int m_classID{-1};
        std::string m_className{""};
        double m_confidence{0};
    };

    class VisionResultsClass
    {
    public:
        VisionResultsClass() {}
        ~VisionResultsClass() {}
        std::vector<RetroreflectiveResultClass> RetroResults;
        std::vector<FiducialResultClass> FiducialResults;
        std::vector<DetectionResultClass> DetectionResults;
        std::vector<ClassificationResultClass> ClassificationResults;
        double m_timeStamp{-1.0};
        double m_latencyPipeline{0};
        double m_latencyCapture{0};
        double m_latencyJSON{0};
        double m_pipelineIndex{-1.0};
        int valid{0};
        std::vector<double> botPose{6,0.0};
        std::vector<double> botPose_wpired{6,0.0};
        std::vector<double> botPose_wpiblue{6,0.0};
        void Clear()
        {
            RetroResults.clear();
            FiducialResults.clear();
            DetectionResults.clear();
            ClassificationResults.clear();
            botPose.clear();
            botPose_wpired.clear();
            botPose_wpiblue.clear();
            m_timeStamp = -1.0;
            m_latencyPipeline = 0;

            m_pipelineIndex = -1.0;
        }
    };

    class LimelightResultsClass
    {
    public:
        LimelightResultsClass() {}
        ~LimelightResultsClass() {}
        VisionResultsClass targetingResults;
    };

    namespace internal
    {
        inline const std::string _key_timestamp{"ts"};
        inline const std::string _key_latency_pipeline{"tl"};
        inline const std::string _key_latency_capture{"cl"};

        inline const std::string _key_pipelineIndex{"pID"};
        inline const std::string _key_TargetXDegrees{"txdr"};
        inline const std::string _key_TargetYDegrees{"tydr"};
        inline const std::string _key_TargetXNormalized{"txnr"};
        inline const std::string _key_TargetYNormalized{"tynr"};
        inline const std::string _key_TargetXPixels{"txp"};
        inline const std::string _key_TargetYPixels{"typ"};

        inline const std::string _key_TargetXDegreesCrosshair{"tx"};
        inline const std::string _key_TargetYDegreesCrosshair{"ty"};
        inline const std::string _key_TargetXNormalizedCrosshair{"txn"};
        inline const std::string _key_TargetYNormalizedCrosshair{"tyn"};
        inline const std::string _key_TargetAreaNormalized{"ta"};
        inline const std::string _key_TargetAreaPixels{"tap"};
        inline const std::string _key_className{"class"};
        inline const std::string _key_classID{"classID"};
        inline const std::string _key_confidence{"conf"};
        inline const std::string _key_fiducialID{"fID"};
        inline const std::string _key_corners{"pts"};
        inline const std::string _key_transformCAMERAPOSE_TARGETSPACE{"t6c_ts"};
        inline const std::string _key_transformCAMERAPOSE_ROBOTSPACE{"t6c_rs"};

        inline const std::string _key_transformTARGETPOSE_CAMERASPACE{"t6t_cs"};
        inline const std::string _key_transformROBOTPOSE_TARGETSPACE{"t6r_ts"};
        inline const std::string _key_transformTARGETPOSE_ROBOTSPACE{"t6t_rs"};

        inline const std::string _key_botpose{"botpose"};
        inline const std::string _key_botpose_wpiblue{"botpose_wpiblue"};
        inline const std::string _key_botpose_wpired{"botpose_wpired"};

        inline const std::string _key_transformROBOTPOSE_FIELDSPACE{"t6r_fs"};
        inline const std::string _key_skew{"skew"};
        inline const std::string _key_ffamily{"fam"};
        inline const std::string _key_colorRGB{"cRGB"};
        inline const std::string _key_colorHSV{"cHSV"};
    }

    // inline void PhoneHome() 
    // {
    //     static int sockfd = -1;
    //     static struct sockaddr_in servaddr, cliaddr;

    //     if (sockfd == -1) {
    //         sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    //         if (sockfd < 0) {
    //             std::cerr << "Socket creation failed" << std::endl;
    //             return;
    //         }

    //         memset(&servaddr, 0, sizeof(servaddr));
    //         servaddr.sin_family = AF_INET;
    //         servaddr.sin_addr.s_addr = inet_addr("255.255.255.255");
    //         servaddr.sin_port = htons(5809);

    //         // Set socket for broadcast
    //         int broadcast = 1;
    //         if (setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0) {
    //             std::cerr << "Error in setting Broadcast option" << std::endl;
    //             close(sockfd);
    //             sockfd = -1;
    //             return;
    //         }

    //         // Set socket to non-blocking
    //         if (fcntl(sockfd, F_SETFL, O_NONBLOCK) < 0) {
    //             std::cerr << "Error setting socket to non-blocking" << std::endl;
    //             close(sockfd);
    //             sockfd = -1;
    //             return;
    //         }

    //         const char *msg = "LLPhoneHome";
    //         sendto(sockfd, msg, strlen(msg), 0, (const struct sockaddr *) &servaddr, sizeof(servaddr));
    //     }

    //     char receiveData[1024];
    //     socklen_t len = sizeof(cliaddr);

    //     ssize_t n = recvfrom(sockfd, (char *)receiveData, 1024, 0, (struct sockaddr *) &cliaddr, &len);
    //     if (n > 0) {
    //         receiveData[n] = '\0'; // Null-terminate the received string
    //         std::string received(receiveData, n);
    //         std::cout << "Received response: " << received << std::endl;
    //     } else if (n < 0 && errno != EWOULDBLOCK && errno != EAGAIN) {
    //         std::cerr << "Error receiving data" << std::endl;
    //         close(sockfd);
    //         sockfd = -1;
    //     }
    // }

    inline void SetupPortForwarding(const std::string& limelightName) 
    {
        auto& portForwarder = wpi::PortForwarder::GetInstance();
        portForwarder.Add(5800, sanitizeName(limelightName), 5800);
        portForwarder.Add(5801, sanitizeName(limelightName), 5801);
        portForwarder.Add(5802, sanitizeName(limelightName), 5802);
        portForwarder.Add(5803, sanitizeName(limelightName), 5803);
        portForwarder.Add(5804, sanitizeName(limelightName), 5804);
        portForwarder.Add(5805, sanitizeName(limelightName), 5805);
        portForwarder.Add(5806, sanitizeName(limelightName), 5806);
        portForwarder.Add(5807, sanitizeName(limelightName), 5807);
        portForwarder.Add(5808, sanitizeName(limelightName), 5808);
        portForwarder.Add(5809, sanitizeName(limelightName), 5809);
    }

    template <typename T, typename KeyType>
    T SafeJSONAccess(const wpi::json& jsonData, const KeyType& key, const T& defaultValue)
    {
        try
        {
           return jsonData.at(key).template get<T>();
        }
        catch (wpi::json::exception&)
        {
            return defaultValue;
        }
        catch (...)
        {
            return defaultValue;
        }
    }
    inline void from_json(const wpi::json &j, RetroreflectiveResultClass &t)
    {
        std::vector<double> defaultValueVector(6, 0.0);
        t.m_CAMERATransform6DTARGETSPACE = SafeJSONAccess<std::vector<double>>(j, internal::_key_transformCAMERAPOSE_TARGETSPACE, defaultValueVector);
        t.m_CAMERATransform6DROBOTSPACE = SafeJSONAccess<std::vector<double>>(j, internal::_key_transformCAMERAPOSE_ROBOTSPACE, defaultValueVector);

        t.m_TargetTransform6DCAMERASPACE = SafeJSONAccess<std::vector<double>>(j, internal::_key_transformTARGETPOSE_CAMERASPACE, defaultValueVector);
        t.m_TargetTransform6DROBOTSPACE = SafeJSONAccess<std::vector<double>>(j, internal::_key_transformTARGETPOSE_ROBOTSPACE, defaultValueVector);
        t.m_ROBOTTransform6DTARGETSPACE = SafeJSONAccess<std::vector<double>>(j, internal::_key_transformROBOTPOSE_TARGETSPACE, defaultValueVector);
        t.m_ROBOTTransform6DFIELDSPACE = SafeJSONAccess<std::vector<double>>(j, internal::_key_transformROBOTPOSE_FIELDSPACE, defaultValueVector);

        t.m_TargetXPixels = SafeJSONAccess<double>(j, internal::_key_TargetXPixels, 0.0);
        t.m_TargetYPixels = SafeJSONAccess<double>(j, internal::_key_TargetYPixels, 0.0);
        t.m_TargetXDegreesCrosshairAdjusted = SafeJSONAccess<double>(j, internal::_key_TargetXDegreesCrosshair, 0.0);
        t.m_TargetYDegreesCrosshairAdjusted = SafeJSONAccess<double>(j, internal::_key_TargetYDegreesCrosshair, 0.0);
        t.m_TargetAreaNormalized = SafeJSONAccess<double>(j, internal::_key_TargetAreaNormalized, 0.0);
        t.m_TargetCorners = SafeJSONAccess<std::vector<std::vector<double>>>(j, internal::_key_corners, std::vector<std::vector<double>>{});
    }

    inline void from_json(const wpi::json &j,  FiducialResultClass &t)
    {
        std::vector<double> defaultValueVector(6, 0.0);
        t.m_family = SafeJSONAccess<std::string>(j, internal::_key_ffamily, "");
        t.m_fiducialID = SafeJSONAccess<double>(j, internal::_key_fiducialID, 0.0);
        t.m_CAMERATransform6DTARGETSPACE = SafeJSONAccess<std::vector<double>>(j, internal::_key_transformCAMERAPOSE_TARGETSPACE, defaultValueVector);
        t.m_CAMERATransform6DROBOTSPACE = SafeJSONAccess<std::vector<double>>(j, internal::_key_transformCAMERAPOSE_ROBOTSPACE, defaultValueVector);
        t.m_TargetTransform6DCAMERASPACE = SafeJSONAccess<std::vector<double>>(j, internal::_key_transformTARGETPOSE_CAMERASPACE, defaultValueVector);
        t.m_TargetTransform6DROBOTSPACE = SafeJSONAccess<std::vector<double>>(j, internal::_key_transformTARGETPOSE_ROBOTSPACE, defaultValueVector);
        t.m_ROBOTTransform6DTARGETSPACE = SafeJSONAccess<std::vector<double>>(j, internal::_key_transformROBOTPOSE_TARGETSPACE, defaultValueVector);
        t.m_ROBOTTransform6DFIELDSPACE = SafeJSONAccess<std::vector<double>>(j, internal::_key_transformROBOTPOSE_FIELDSPACE, defaultValueVector);

        t.m_TargetXPixels = SafeJSONAccess<double>(j, internal::_key_TargetXPixels, 0.0);
        t.m_TargetYPixels = SafeJSONAccess<double>(j, internal::_key_TargetYPixels, 0.0);
        t.m_TargetXDegreesCrosshairAdjusted = SafeJSONAccess<double>(j, internal::_key_TargetXDegreesCrosshair, 0.0);
        t.m_TargetYDegreesCrosshairAdjusted = SafeJSONAccess<double>(j, internal::_key_TargetYDegreesCrosshair, 0.0);
        t.m_TargetAreaNormalized = SafeJSONAccess<double>(j, internal::_key_TargetAreaNormalized, 0.0);
        t.m_TargetCorners = SafeJSONAccess<std::vector<std::vector<double>>>(j, internal::_key_corners, std::vector<std::vector<double>>{});
    }

    inline void from_json(const wpi::json &j,  DetectionResultClass &t)
    {
        t.m_confidence = SafeJSONAccess<double>(j, internal::_key_confidence, 0.0);
        t.m_classID = SafeJSONAccess<double>(j, internal::_key_classID, 0.0);
        t.m_className = SafeJSONAccess<std::string>(j, internal::_key_className, "");
        t.m_TargetXPixels = SafeJSONAccess<double>(j, internal::_key_TargetXPixels, 0.0);
        t.m_TargetYPixels = SafeJSONAccess<double>(j, internal::_key_TargetYPixels, 0.0);
        t.m_TargetXDegreesCrosshairAdjusted = SafeJSONAccess<double>(j, internal::_key_TargetXDegreesCrosshair, 0.0);
        t.m_TargetYDegreesCrosshairAdjusted = SafeJSONAccess<double>(j, internal::_key_TargetYDegreesCrosshair, 0.0);
        t.m_TargetAreaNormalized = SafeJSONAccess<double>(j, internal::_key_TargetAreaNormalized, 0.0);
        t.m_TargetCorners = SafeJSONAccess<std::vector<std::vector<double>>>(j, internal::_key_corners, std::vector<std::vector<double>>{});
    }

    inline void from_json(const wpi::json &j,  ClassificationResultClass &t)
    {
        t.m_confidence = SafeJSONAccess<double>(j, internal::_key_confidence, 0.0);
        t.m_classID = SafeJSONAccess<double>(j, internal::_key_classID, 0.0);
        t.m_className = SafeJSONAccess<std::string>(j, internal::_key_className, "");
        t.m_TargetXPixels = SafeJSONAccess<double>(j, internal::_key_TargetXPixels, 0.0);
        t.m_TargetYPixels = SafeJSONAccess<double>(j, internal::_key_TargetYPixels, 0.0);
        t.m_TargetXDegreesCrosshairAdjusted = SafeJSONAccess<double>(j, internal::_key_TargetXDegreesCrosshair, 0.0);
        t.m_TargetYDegreesCrosshairAdjusted = SafeJSONAccess<double>(j, internal::_key_TargetYDegreesCrosshair, 0.0);
        t.m_TargetAreaNormalized = SafeJSONAccess<double>(j, internal::_key_TargetAreaNormalized, 0.0);
        t.m_TargetCorners = SafeJSONAccess<std::vector<std::vector<double>>>(j, internal::_key_corners, std::vector<std::vector<double>>{});
    }

    inline void from_json(const wpi::json &j,  VisionResultsClass &t)
    {
        t.m_timeStamp = SafeJSONAccess<double>(j, internal::_key_timestamp, 0.0);
        t.m_latencyPipeline = SafeJSONAccess<double>(j, internal::_key_latency_pipeline, 0.0);
        t.m_latencyCapture = SafeJSONAccess<double>(j, internal::_key_latency_capture, 0.0);
        t.m_pipelineIndex = SafeJSONAccess<double>(j, internal::_key_pipelineIndex, 0.0);
        t.valid = SafeJSONAccess<double>(j, "v", 0.0);

        std::vector<double> defaultVector{};
        t.botPose = SafeJSONAccess<std::vector<double>>(j, internal::_key_botpose, defaultVector);
        t.botPose_wpired = SafeJSONAccess<std::vector<double>>(j, internal::_key_botpose_wpired, defaultVector);
        t.botPose_wpiblue = SafeJSONAccess<std::vector<double>>(j, internal::_key_botpose_wpiblue, defaultVector);

        t.RetroResults = SafeJSONAccess<std::vector< RetroreflectiveResultClass>>(j, "Retro", std::vector< RetroreflectiveResultClass>{});
        t.FiducialResults = SafeJSONAccess<std::vector< FiducialResultClass>>(j, "Fiducial", std::vector< FiducialResultClass>{});
        t.DetectionResults = SafeJSONAccess<std::vector< DetectionResultClass>>(j, "Detector", std::vector< DetectionResultClass>{});
        t.ClassificationResults = SafeJSONAccess<std::vector< ClassificationResultClass>>(j, "Detector", std::vector< ClassificationResultClass>{});
    }

    inline void from_json(const wpi::json &j,  LimelightResultsClass &t)
    {
        t.targetingResults = SafeJSONAccess<LimelightHelpers::VisionResultsClass>(j, "Results",  LimelightHelpers::VisionResultsClass{});
    }

    inline LimelightResultsClass getLatestResults(const std::string &limelightName = "", bool profile = false)
    {
        auto start = std::chrono::high_resolution_clock::now();
        std::string jsonString = getJSONDump(limelightName); 
        wpi::json data;
        try
        {
            data = wpi::json::parse(jsonString);
        }
        catch(const std::exception&)
        {
           return LimelightResultsClass();
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        double nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        double millis = (nanos * 0.000001);
        try
        {
            LimelightResultsClass out = data.get<LimelightHelpers::LimelightResultsClass>();
            out.targetingResults.m_latencyJSON = millis;
            if (profile)
            {
                std::cout << "lljson: " << millis << std::endl;
            }
            return out;
        }
        catch(...) 
        {
            return LimelightResultsClass();
        }
    }
}
