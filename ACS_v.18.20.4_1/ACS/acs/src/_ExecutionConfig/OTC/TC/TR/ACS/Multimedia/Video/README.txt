This is a README file on the TestCases for the Video Quality Playback check.

These TCs are based on the LIVE_VIDEO_QUALITY.py usecase located in ../../acs_test_scripts/UseCase/OTC/Media

The way this UseCase works is as follows:
1. Retrieve needed video from Artifactory and push it on the device(done in the <Initialize> node, via TestSteps)
2. Play the video by it's filename
3. Record the playback via the 'adb screenrecord' command and store it on the device
4. Pull the recording from the device onto the host
5. Compare the screenrecord file with a reference file on the host(also downloadable from the Artifactory) on a frame by frame basis. This comparison is being done based on the python file:
    ../acs_test_scripts/Lib/VideoCheck/VideoCheck.py through the compare_two_video_frame_by_frame() method

This method takes several parameters into account, needed for the algorithms it calls upon.
As the test is intended to check for frame by frame quality playback, the default threshold parameters are quite small, leaving little place for error.
In order to adapt to the various platform stabilities at hand, there are 3 parameters for this method that can be adjusted in accordance to the video playback performance that is considered suitable for the instance. This is due to the fact that instable platforms will fail the tests with the default values.
These values are to be modified in the usecase file ../../acs_test_scripts/UseCase/OTC/Media/LIVE_VIDEO_QUALITY.py
The parameters in question that are to be modified and increased accordingly are the following:
1. max_consecutive_missing_frame
    default value: 5
    - found in the main method for comparison.
    - it's value tells the algorithm how many consecutive frames can be missing from the tested file and the test still be considered a success.
2. percent_threshold
    default value: 0.10
    - found in the get_percent_diff helper method. It is the maximum allowed difference between 2 frames found on the same position in the file.
    - With the default value, for instance, the added navbars are not faulting the result.
    - Recommended to be higher on more instable platforms.
3. match_template_threshold
    default value: 0.99
    - found in the match_template_on_video_frame helper method. This method takes the reference video file as the template and then matches the tested file against it's frames.
    - with the default value of 99%, it checks for nearly identical frames.
