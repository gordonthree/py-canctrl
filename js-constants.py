import pandas as pd

def generate_js_constants(csv_filename, js_filename):
    # Defining constants for CSV structure to avoid magic numbers and improve readability
    HEADER_ROW_INDEX = 5 # Data headers are located on row 6 of the CSV
    COL_IDENTIFIER = 'c def' # Column name containing the message identifier
    COL_MESSAGE_ID = 'Message ID' # Column name for the hex ID
    COL_DLC = 'DLC' # Column name for the Data Length Code
    COL_COMMENTS = 'Comments' # Column name for the message description

    # Load the CSV, starting at the identified header row
    df = pd.read_csv(csv_filename, header=HEADER_ROW_INDEX)

    # Sanitize column names by removing any internal newlines found in the CSV
    df.columns = [col.replace('\n', ' ') for col in df.columns]

    # Filter out rows where the message identifier is empty or invalid
    df = df.dropna(subset=[COL_IDENTIFIER])

    js_lines = []
    for _, row in df.iterrows():
        base_name = str(row[COL_IDENTIFIER]).strip()
        msg_id = str(row[COL_MESSAGE_ID]).strip()
        
        # Clean the comment and handle empty values
        comment = str(row[COL_COMMENTS]).strip() if pd.notna(row[COL_COMMENTS]) else ""

        # Normalize the DLC value to an integer string
        try:
            dlc_val = int(float(row[COL_DLC]))
            dlc = str(dlc_val)
        except (ValueError, TypeError):
            dlc = str(row[COL_DLC]).strip()

        # Skip rows with placeholders or non-identifier data
        if not base_name or base_name.lower() in ['nan', 'c def']:
            continue

        # Generate the ID constant with Doxygen style trailing comment
        id_line = f"export const {base_name}_ID = {msg_id};"
        if comment:
            id_line = f" /** {msg_id} {comment} */\n" + id_line
        
        js_lines.append(id_line)
        
        # Generate the DLC constant
        js_lines.append(f"export const {base_name}_DLC = {dlc};")

    # Save the formatted constants to the output JavaScript file
    with open(js_filename, 'w') as f:
        f.write("\n".join(js_lines))

if __name__ == "__main__":
    CSV_INPUT = 'can bus messages - Messages.csv'
    JS_OUTPUT = 'can_constants.js'
    
    generate_js_constants(CSV_INPUT, JS_OUTPUT)
    print(f"JavaScript constants generated in {JS_OUTPUT}")
    print("Hint: import * as CAN from './can_constants.js';")
    