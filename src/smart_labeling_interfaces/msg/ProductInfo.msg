# Based on the columns in products.csv
# This message carries the ground-truth data for a single product.

string device_id
string batch_id
string manufacturing_date # We'll keep this as a string for simplicity
bool rohs_compliant
string expected_serial_number_qr

# We'll add the other expected values here too for the validator node
string expected_batch_ocr
string expected_serial_ocr