#!/usr/bin/env python
import numpy as np

class PotentialField(object):
    def __init__ (self):
        self.K_a        = 5     # Attractive potential gain
        self.K_r        = 100   # Repulsive potential gain
        self.area_width = 0.2  # Potential area width
    
    def calc_potential_field(self, gx, gy, gz, ox, oy, oz, reso, rho_0):
        # Inputs: 
        # gx, gy, yz: position of the goal (doubles)
        # ox, oy, oz: position of the obstacles
        # reso:       potential map grid size
        # rho_0:      joint / end effector size

        # All the points and obstacles are in this region:
        minx = -0.5
        miny = -0.5
        minz = 0
        maxx = 0.5
        maxy = 0.5
        maxz = 0.7

        # Workspace size (points):
        xw   = int(round((maxx - minx) / reso))
        yw   = int(round((maxy - miny) / reso))
        zw   = int(round((maxz - minz) / reso))
        
        
        # Potential map
        pmap = [[[0.0 for i in range(zw)] for i in range(yw)] for i in range(xw)]
        
        # Calculate potential for each unit position:
        for ix in range(xw):
            x = ix * reso + minx
            for iy in range(yw):
                y = iy * reso + miny
                for iz in range(zw):
                    z = iz * reso + minz
                    print(xw, yw, zw)
                    print(ix, iy, iz)
                    ua = self.attractive_potential(x, y, z, gx, gy, gz)
                    print(ua)
                    ur = self.repulsive_potential(x, y, z, ox, oy, oz, rho_0)
                    print(ur)
                    u = ua + ur
                    pmap[ix][iy][iz] = u
        
        return pmap, minx, miny, minz


    def attractive_potential(self, x, y, z, gx, gy, gz):
        # Input:
        # x, y, z: arbitrary position in workspace
        # gx, gy, gz: goal position
        # Since the movement is always within the distance between the starting and the goal position:
        return 0.5 * self.K_a * np.sqrt((x - gx)**2 + (y - gy)**2 + (z - gz)**2)**2


    def repulsive_potential(self, x, y, z, ox, oy, oz, rho_0):
        # Initialize:
        minid = -1
        dmin  = float("inf")

        for i, dont_care in enumerate(ox):
            d = np.sqrt((x - ox[i])**2 + (y - oy[i])**2 + (z - oz[i])**2)
            if dmin >= d:
                dmin = d
                minid = i

        # Shortest distance the point [x y z] and the nearest obstacle:
        rho_q = np.sqrt((x - ox[minid])**2 + (y - oy[minid])**2 + (z - oz[minid])**2)

        if rho_q <= rho_0:
            return 0.5*self.K_r*(1/rho_q - 1/rho_0)**2
        else:
            return 0


    def get_moves(self):
        # Possible moves at every step:
        # dx, dy, dz
        moves = [
            # z = 0:
            [-1,  1,  0],
            [-1,  0,  0],
            [-1, -1,  0],
            [ 0,  1,  0],
            [ 0, -1,  0],
            [ 1,  1,  0],
            [ 1,  0,  0],
            [ 1, -1,  0],
            # z = 1: 
            [-1,  1,  1],
            [-1,  0,  1],
            [-1, -1,  1],
            [ 0,  1,  1],
            [ 0,  0,  1],
            [ 0, -1,  1],
            [ 1,  1,  1],
            [ 1,  0,  1],
            [ 1, -1,  1],
            # z = -1:
            [-1,  1, -1],
            [-1,  0, -1],
            [-1, -1, -1],
            [ 0,  1, -1],
            [ 0,  0, -1],
            [ 0, -1, -1],
            [ 1,  1, -1],
            [ 1,  0, -1],
            [ 1, -1, -1],
        ]
        return moves


    def potential_field_planning(self, sx, sy, sz, gx, gy, gz, ox, oy, oz, reso, rho_0):
        # Get potential field map
        pmap, minx, miny, minz = self.calc_potential_field(gx, gy, gz, ox, oy, oz, reso, rho_0)

        # Distance between the start and goal position:
        d   = np.sqrt((sx - gx)**2 + (sy - gy)**2 + (sz - gz)**2)

        # Starting position:
        ix  = round((sx - minx) / reso)
        iy  = round((sy - miny) / reso)
        iz  = round((sz - minz) / reso)

        # Initialize path
        rx, ry, rz = [sx], [sy], [sz]
        moves      = self.get_moves()
        
        # Work while the distance between the moving position 
        # and the goal is larger than 1 unit of move:
        while d >= reso:
            # Initialize:
            minp = float("inf")
            minix, miniy, miniz = -1, -1, -1

            # Get the move that can have the minimum potential:
            for i, _ in enumerate(moves):
                # Try every move:
                inx = int(ix + moves[i][0])
                iny = int(iy + moves[i][1])
                inz = int(iz + moves[i][2])

                # Get potential in the potential map:
                if inx >= len(pmap) or iny >= len(pmap[0]) or inz >= len(pmap[0][0]):
                    # Outside the ROI
                    p = float("inf")
                else:
                    p = pmap[inx][iny][inz]
                
                # Compare potentials:
                if minp > p:
                    minp  = p
                    minix = inx
                    miniy = iny
                    miniz = inz
            
            # Overwrite the current position in the map to
            # the one with minimum potential:
            ix = minix
            iy = miniy
            iz = miniz

            # Get the actual position:
            xp = ix * reso + minx
            yp = iy * reso + miny
            zp = iz * reso + minz

            # New distance between the goal and the current position:
            d  = np.sqrt((gx - xp)**2 + (gy - yp)**2 + (gz - zp)**2)
            
            # Append actual position to path:
            rx.append(xp)
            ry.append(yp)
            rz.append(zp)

        return rx, ry, rz