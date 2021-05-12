
def init_tower(index):
    if index <= 3:
        return [0., 0., 1.57, 0.6, 0., 0.06]
    return [0., 0., 1., 0.6, 0., 0.06]


def init_disks(index):
    disk_positions = []
    if index == 0:
        # big disk
        disk_positions.append([-1.96599566e-06, 1.51755420e-05, -2.92120386e+00, 5.99999823e-01, 1.99999779e-01, 5.75006475e-02])
        # medium disk
        disk_positions.append([-1.60976100e-05, 1.30302899e-05, -2.90417380e+00, 6.04137783e-01, 1.98690331e-01, 8.25017041e-02])
        # small disk
        disk_positions.append([-4.81728662e-05, -3.90959381e-05, -2.85433699e+00, 5.99475488e-01, 1.97810570e-01, 1.07506470e-01])
        return disk_positions
    elif index == 1:
        # big disk
        disk_positions.append([-3.37608025e-06, 1.39789516e-05, -2.91876550e+00, 5.99873214e-01, -3.92385188e-04, 5.75006725e-02])
        # medium disk
        disk_positions.append([7.17317564e-06, 2.12674252e-05, -2.99694570e+00, 6.03504413e-01, 7.06402998e-03, 8.25003996e-02])
        # small disk
        disk_positions.append([1.84741021e-03, -1.02394461e-03, 3.14132934e+00, 6.04143948e-01, 2.60369318e-03, 1.07488283e-01])
        return disk_positions
    elif index == 2:
        # big disk
        disk_positions.append([-3.37608025e-06, 1.39789516e-05, -2.91876550e+00, 5.99873214e-01, -3.92385188e-04, 5.75006725e-02])
        # medium disk
        disk_positions.append([7.17317564e-06, 2.12674252e-05, -2.99694570e+00, 6.03504413e-01, 7.06402998e-03, 8.25003996e-02])
        # small disk
        disk_positions.append([7.88643397e-05, -8.11211289e-05, -2.92120397e+00, 6.01040976e-01, 1.97545140e-01, 5.75047890e-02])
        return disk_positions
    elif index == 3:
        # big disk
        disk_positions.append([-4.74325642e-06, 1.27593240e-05, -2.91999408e+00, 5.99598289e-01, -1.63656791e-03, 5.75007966e-02])
        # medium disk
        disk_positions.append([-4.81214699e-06, 2.25556827e-05, -2.92120397e+00, 5.99840795e-01, -2.00000120e-01, 5.75010458e-02])
        # small disk
        disk_positions.append([-1.89896344e-04, -4.59067973e-05, 2.93037524e+00, 6.04636754e-01, 5.72662647e-03, 8.25020359e-02])
        return disk_positions
    else:
        # big disk
        disk_positions.append([-4.74325642e-06, 1.27593240e-05, -2.91999408e+00, 5.99598289e-01, -1.63656791e-03, 5.75007966e-02])
        # medium disk
        disk_positions.append([-4.71556820e-06, 2.25843229e-05, -2.92120397e+00, 7.08060521e-01, 1.68294013e-01, 5.75009531e-02])
        # small disk
        disk_positions.append([-1.89896344e-04, -4.59067973e-05, 2.93037524e+00, 6.04636754e-01, 5.72662647e-03, 8.25020359e-02])
        return disk_positions
