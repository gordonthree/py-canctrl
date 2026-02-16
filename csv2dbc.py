#!/bin/python3
import pandas as pd
import cantools.database
import re

def clean_name(name):
    # Cleans strings to be valid DBC names (alphanumeric + underscores)
    name = re.sub(r'[^a-zA-Z0-9]', '_', str(name))
    name = re.sub(r'_+', '_', name)
    name = name.strip('_')
    return name[:32] # Limit to 32 chars for compatibility

def csv_to_dbc(csv_path, dbc_output):
    # Load CSV - skipping the first 5 header/empty rows
    df = pd.read_csv(csv_path, skiprows=5)
    
    # Clean column names (removes newlines from headers)
    df.columns = [c.replace('\n', ' ') for c in df.columns]
    
    db = cantools.database.can.Database()
    used_msg_names = set()
    
    for _, row in df.iterrows():
        # Skip rows without a valid Message ID or C-definition name
        if pd.isna(row['Message ID']) or pd.isna(row['c def']):
            continue
            
        try:
            # Parse core message properties
            msg_id = int(str(row['Message ID']).strip(), 16)
            
            # Ensure unique and clean message names
            base_msg_name = clean_name(row['c def'])
            msg_name = base_msg_name
            if msg_name in used_msg_names:
                msg_name = f"{base_msg_name}_{msg_id:X}"
            used_msg_names.add(msg_name)
            
            # Default to DLC 8 if not specified
            dlc = int(row['DLC']) if not pd.isna(row['DLC']) else 8
            
            signals = []
            payload_labels = []
            for i in range(8):
                col = f"D{i} B{i+1}"
                val = str(row[col]).strip() if col in df.columns and not pd.isna(row[col]) else None
                if not val or val.lower() == 'nan' or val == '':
                    val = None
                payload_labels.append(val)

            i = 0
            used_sig_names = set()
            while i < dlc:
                label = payload_labels[i]
                
                if label is None:
                    i += 1
                    continue
                
                # Group consecutive identical labels into one multi-byte signal
                count = 1
                while (i + count < dlc) and (payload_labels[i + count] == label):
                    count += 1
                
                # Create a unique, clean signal name
                base_sig_name = clean_name(label)
                sig_name = base_sig_name
                suffix = 1
                while sig_name in used_sig_names:
                    suffix_str = f"_{suffix}"
                    sig_name = base_sig_name[:32-len(suffix_str)] + suffix_str
                    suffix += 1
                used_sig_names.add(sig_name)
                
                # BIG ENDIAN (Motorola) MSB Logic:
                # In standard CAN bit numbering, MSB of byte 'i' is (i * 8) + 7.
                # Example: Byte 0 MSB = 7, Byte 5 MSB = 47.
                start_bit = (i * 8) + 7
                
                signals.append(cantools.database.can.Signal(
                    name=sig_name,
                    start=start_bit,
                    length=count * 8,
                    byte_order='big_endian'
                ))
                
                i += count

            # Create the message and add to the database
            message = cantools.database.can.Message(
                frame_id=msg_id,
                name=msg_name,
                length=dlc,
                signals=signals,
                comment=str(row['Comments']) if not pd.isna(row['Comments']) else None
            )
            db.messages.append(message)
            
        except (ValueError, TypeError, Exception) as e:
            # print(f"Skipping row for {row.get('c def')}: {e}")
            continue

    # Export the database to the DBC file
    # This method is the most compatible across cantools versions
    cantools.database.dump_file(db, dbc_output)
    
    print(f"Successfully generated {dbc_output}")

# Execute
csv_to_dbc('can bus messages - Messages.csv', 'master_bus.dbc')
