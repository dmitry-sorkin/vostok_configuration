#!/usr/bin/env python3
"""
G-code processor script.
Processes G-code files to remove specific extrusion commands, consecutive duplicate commands,
and add deretract commands.
"""

import argparse
import re
import sys
from pathlib import Path


def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description='Process G-code file to remove G1 E* commands, consecutive duplicate commands, and add deretract command.'
    )
    parser.add_argument(
        'file_path',
        type=str,
        help='Path to the G-code file to process'
    )
    parser.add_argument(
        '--long_deretract_length',
        type=float,
        default=None,
        help='Length for deretract command (E value). If not provided, no deretract command is added.'
    )
    return parser.parse_args()


def is_t_x_y_line(line):
    """
    Check if line matches pattern T... X... Y...
    Pattern: T followed by numbers, then X followed by numbers, then Y followed by numbers.
    """
    stripped = line.strip()
    pattern = r'^T\d+\s+X[-+]?[\d.]+\s+Y[-+]?[\d.]+'
    return bool(re.match(pattern, stripped))


def is_type_comment(line):
    """Check if line is a ;TYPE:* comment."""
    stripped = line.strip()
    return stripped.startswith(';TYPE:')


def is_g1_e_only(line):
    """
    Check if line is G1 E* command (only G1 and E, no other parameters between them).
    Examples:
    - "G1 E10" -> True
    - "G1 E10 F1500" -> True (F is after E)
    - "G1 X10 E10" -> False (X is between G1 and E)
    """
    stripped = line.strip()
    
    if not stripped.startswith('G1'):
        return False
    
    after_g1 = stripped[2:]
    after_g1_stripped = after_g1.lstrip()
    
    if not after_g1_stripped:
        return False
    
    if not after_g1_stripped.startswith('E'):
        return False
    
    after_e = after_g1_stripped[1:]
    
    if not after_e:
        return False
    
    number_pattern = r'^\s*[-+]?[\d.]+'
    if not re.match(number_pattern, after_e):
        return False
    
    e_pos = after_g1.find('E')
    if e_pos == -1:
        return False
    
    before_e = after_g1[:e_pos]
    if before_e and not before_e.isspace():
        return False
    
    return True


def remove_consecutive_duplicates(segment_lines):
    """
    Remove consecutive duplicate commands from segment lines.
    Keeps only the first occurrence of each group of consecutive identical commands.
    
    Args:
        segment_lines: List of lines in the segment (after other removals)
    
    Returns:
        List of lines with consecutive duplicates removed
    """
    if not segment_lines:
        return segment_lines
    
    result = [segment_lines[0]]  # Keep first line
    
    for i in range(1, len(segment_lines)):
        current_stripped = segment_lines[i].strip()
        # Compare with the LAST line added to result (not previous in original list)
        last_added_stripped = result[-1].strip()
        
        # Skip if current line is identical to last added line
        if current_stripped == last_added_stripped:
            continue
        
        result.append(segment_lines[i])
    
    return result


def find_all_segments(lines):
    """
    Find all segments starting with T... X... Y... and ending with ;TYPE:*
    Returns list of tuples (start_idx, end_idx, t_line) where:
    - start_idx: index of T... X... Y... line
    - end_idx: index of ;TYPE:* line (or len(lines) if not found)
    - t_line: the actual T line content for coordinate extraction
    """
    segments = []
    i = 0
    
    while i < len(lines):
        if is_t_x_y_line(lines[i]):
            start_idx = i
            t_line = lines[i]
            
            # Find the end of this segment (next ;TYPE:* or end of file)
            end_idx = len(lines)
            for j in range(i + 1, len(lines)):
                if is_type_comment(lines[j]):
                    end_idx = j
                    break
            
            segments.append((start_idx, end_idx, t_line))
            i = end_idx  # Continue searching from the end of this segment
        else:
            i += 1
    
    return segments


def process_gcode(file_path, long_deretract_length):
    """
    Process G-code file according to specifications.
    
    Args:
        file_path: Path to G-code file
        long_deretract_length: E value for deretract command (None to skip)
    
    Returns:
        True if processing successful, False otherwise
    """
    path = Path(file_path)
    
    if not path.exists():
        print(f"Error: File '{file_path}' not found.", file=sys.stderr)
        return False
    
    try:
        # Read all lines
        with open(path, 'r') as f:
            lines = f.readlines()
        
        # Find all segments
        segments = find_all_segments(lines)
        
        if not segments:
            print("No T... X... Y... segments found. Nothing to process.")
            return True
        
        print(f"Found {len(segments)} segment(s) to process")
        
        # Track which lines to remove and where to insert deretract
        # For each segment: (start_idx, end_idx, lines_to_remove_set, insert_deretract_before)
        segment_modifications = []
        
        for seg_idx, (start_idx, end_idx, t_line) in enumerate(segments):
            is_first_segment = (seg_idx == 0)
            lines_to_remove = set()
            
            print(f"  Segment {seg_idx + 1}: lines {start_idx + 1}-{end_idx}")
            
            # For first segment only: check for G1 E* commands
            if is_first_segment:
                for i in range(start_idx, end_idx):
                    line = lines[i]
                    if is_g1_e_only(line):
                        lines_to_remove.add(i)
                        print(f"    Removing G1 E* at line {i + 1}: {line.strip()}")
            
            # Determine where to insert deretract (only for first segment)
            insert_deretract_before = None
            if is_first_segment and long_deretract_length is not None:
                insert_deretract_before = end_idx
            
            segment_modifications.append((start_idx, end_idx, lines_to_remove, insert_deretract_before))
        
        # Build new content
        new_lines = []
        current_line = 0
        
        for seg_idx, (start_idx, end_idx, lines_to_remove, insert_deretract_before) in enumerate(segment_modifications):
            # Add lines before this segment
            while current_line < start_idx:
                new_lines.append(lines[current_line])
                current_line += 1
            
            # Collect remaining segment lines (after removals)
            remaining_segment_lines = []
            for i in range(start_idx, end_idx):
                if i not in lines_to_remove:
                    remaining_segment_lines.append(lines[i])
                current_line += 1
            
            # Remove consecutive duplicates from segment
            deduplicated_lines = remove_consecutive_duplicates(remaining_segment_lines)
            
            # Log removed duplicates
            removed_count = len(remaining_segment_lines) - len(deduplicated_lines)
            if removed_count > 0:
                print(f"  Removed {removed_count} consecutive duplicate command(s) from segment {seg_idx + 1}")
            
            # Add deduplicated lines
            new_lines.extend(deduplicated_lines)
            
            # Insert deretract if needed (only for first segment)
            if insert_deretract_before is not None and long_deretract_length is not None:
                deretract_line = f"G1 E{long_deretract_length} F1800\n"
                new_lines.append(deretract_line)
                print(f"  Adding deretract command: {deretract_line.strip()}")
        
        # Add remaining lines after last segment
        while current_line < len(lines):
            new_lines.append(lines[current_line])
            current_line += 1
        
        # Write back to file
        with open(path, 'w') as f:
            f.writelines(new_lines)
        
        print(f"File processed successfully: {file_path}")
        return True
        
    except Exception as e:
        print(f"Error processing file: {e}", file=sys.stderr)
        return False


def main():
    """Main entry point."""
    args = parse_arguments()
    
    success = process_gcode(args.file_path, args.long_deretract_length)
    
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
