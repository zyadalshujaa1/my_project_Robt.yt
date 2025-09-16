import csv

def find_movement(movement_name,file_name):
    # Open the CSV file and search for the matching movement name
    with open(file_name, 'r') as file:
        reader = csv.reader(file)
        row_number = 0
        for row in reader:
            if row[0] == movement_name:
                # Print the row number and angles for the matching movement
                print(f"Row {row_number}: {row}")
                break
            row_number += 1
        else:
            # If no matching movement was found, print an error message
            print("No matching movement found.")
# Ask the user for the movement name
movement_name = input("Enter the movement name: ")
find_movement(movement_name=movement_name,file_name='right_arm.csv')