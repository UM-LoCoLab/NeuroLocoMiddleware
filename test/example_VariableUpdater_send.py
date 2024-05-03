from udpate_vars import send_update

# Prompt the user for a new value for the variable
new_value = float(input("Enter new value for the variable: "))
# Send the new value to the subscriber
send_update(new_value)