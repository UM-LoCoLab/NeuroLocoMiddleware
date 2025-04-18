import sys
import os

# Add the parent folder to the Python path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from VariableUpdater import send_update


def test_float():
    # Prompt the user for a new value for the variable
    new_value = float(input("Enter new value for the variable: "))
    # Send the new value to the subscriber
    send_update(new_value)  

def test_dictionary():
    dictionary = {
        "key1": 4,
        "key2": [1, 2, 3],
        "key3": "test",
    }
    # Send the dictionary to the subscriber
    send_update(dictionary)

if __name__ == "__main__":
    # Test sending a float value
    # test_float()
    # Test sending a dictionary
    test_dictionary()