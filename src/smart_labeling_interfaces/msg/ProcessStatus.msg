# A detailed status message for the UI and other components.

# High-level status text, e.g., "Processing Product", "Process Complete"
string status_text

# A machine-readable overall status
# Can be: "IDLE", "PROCESSING", "COMPLETE", "FAILED"
string overall_status

# smart_labeling_interfaces/msg/ProcessStatus.msg

string product_id
string batch_id
string rohs_status
string image_quality_status
string ml_quality_status # <<< ADD THIS LINE
string qr_status
string ocr_status

