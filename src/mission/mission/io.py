class OutFile:
    def __init__(self, path):
        self.path = path
        self._file = open(self.path, "w")

    def write(self, text):
        self._file.write(text)

    def writeln(self, text):
        self._file.write(text + "\n")

    def writelines(self, *lines):
        self._file.writelines(lines)

    def flush(self):
        self._file.flush()

    def __del__(self):
        if self._file:
            self._file.close()


def dump_file():
    # logger of mapped leaves
    logger = []

    def write_solution_file(self):
        # Generate Plant Numbers
        plant_numbers = [
            f"Plant {letter}{number}" for letter in "AB" for number in range(1, 13)
        ]

        # Initial file writing
        with open("Initial_Mapping_ABE_Gator", "w") as file:
            # Write header
            file.write("Plant Number Healthy Unhealthy Stems Flower\n")

            for entry in logger:
                file.write(
                    f"{plant_numbers[i]} {entry['Healthy']} {entry['Unhealthy']} {entry['Flower']}\n"
                )
            file.close()

        # Final file writing
        with open("Final_Mapping_ABE_Gator", "w") as file:
            # Write header
            file.write("Plant Number Healthy Unhealthy Stems Flower\n")

            for entry, i in zip(logger, range(24)):
                flowers = 0
                if entry["Flower"] >= 1:
                    flowers = 1
                    file.write(f"{plant_numbers[i]} {entry['Healthy']} {0} {flowers}\n")
            file.close()
