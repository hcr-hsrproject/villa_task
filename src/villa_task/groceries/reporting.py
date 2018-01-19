import rospy
from report_generation.msgs import ReportCupboard, ReportCupboardRequest, ReportFinalCupboard, ReportFinalCupboardRequest

report_initial_detections = rospy.ServiceProxy("report_cupboard", ReportCupboard)
report_final_detections = rospy.ServiceProxy("report_final_cupboard", ReportFinalCupboard)


