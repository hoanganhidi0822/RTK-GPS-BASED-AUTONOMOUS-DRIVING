import math

# Function to convert lat/lon to x/y using equirectangular projection
def latlon_to_xy(lat, lon, lat0=10.8532570333, lon0=106.7715131967):
    """
    Convert latitude and longitude to x, y using an equirectangular projection.
    
    lat0 and lon0 represent the center point (origin) for the projection.
    """
    R = 6371e3  # Radius of the Earth in meters
    x = R * math.radians(lon - lon0) * math.cos(math.radians(lat0))
    y = R * math.radians(lat - lat0)
    return x, y

# Read lat/lon from input file
input_file = 'MAP_test.txt'  # Change this to your input file path
output_file = 'output_xyMAP.py'  # Save results as a Python script

wx = []  # List to store x values
wy = []  # List to store y values

with open(input_file, 'r') as infile:
    for line in infile:
        # Strip whitespace and check if the line contains valid data
        line = line.strip()
        if not line:
            continue  # Skip empty lines
        
        try:
            # Split by comma and check if we have both lat and lon
            lat_str, lon_str = line.split(',')
            lat = float(lat_str)
            lon = float(lon_str)
            
            # Convert to x, y coordinates
            x, y = latlon_to_xy(lat, lon)
            
            # Append to lists
            wx.append(x)
            wy.append(y)
        except ValueError:
            print(f"Skipping invalid line: {line}")
            continue  # Skip invalid lines

# Write the result as Python code
with open(output_file, 'w') as outfile:
    outfile.write(f"wx = {wx}\n")
    outfile.write(f"wy = {wy}\n")

print(f"Conversion complete! Results saved in {output_file}")
