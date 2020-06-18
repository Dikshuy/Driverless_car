import optparse
from generateCar import EgoVehicle

def get_options():
    """Parse the controler arguments."""
    optParser = optparse.OptionParser()
    optParser.add_option("--no-enableEgo", dest="enableEgo", action="store_true", default=False,
                         help="runs the command line version of sumo")
    options, args = optParser.parse_args()
    return options


options = get_options()

if options.enableEgo:
    veh = EgoVehicle()
