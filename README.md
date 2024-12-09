# Requirements

- Device running this code needs internet access (to get Unifi camera data)
- Must be run on the same network as the UR10.
- UR10's IP address is hardcoded into `main.py` and `dockermain.py`. Needs to be manualy changed (or rewrite the code so its a env variable instead).
- The UR10 needs enough clearance, as it spins around alot and **WILL** hit things or people otherwise.

### UNIFI_PASSWORD

- **For Docker:** 
    - `UNIFI_PASSWORD` must be set as an environment variable
    - dockerfile is included in the project.
- **For the rest**:
    - `UNIFI_PASSWORD` must be set in a `.env` file. (See `.env.example`).
    - Run `main.py` to run the program.

