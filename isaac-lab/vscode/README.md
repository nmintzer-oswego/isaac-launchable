# Isaac Launchable Usage

This project provides a simple way to interact with Isaac Lab, and Isaac Sim, either on the cloud or locally.
See the [project repo](https://github.com/isaac-sim/isaac-launchable) for more detaild instructions, but below is a quickstart guide.

## Running Isaac Lab 2.3

You can run any of the Issac Lab scripts with the streaming Isaac Sim experience with the following command:

```console
cd /workspace/isaaclab
./isaaclab.sh -p scripts/tutorials/00_sim/create_empty.py --livestream 2
```

Then, in a separate browser window, open `http://{your instance's address}/viewer`.

To run any other Isaac Lab commands, simply append the same argument as shown above: `--livestream 2`

## Running Isaac Sim 5.1

You can run the streaming Isaac Sim application at anytime with the following command.

```console
/isaac-sim/runheadless.sh
```

Or to run and simultaneously accept the EULA:
```console
ACCEPT_EULA=y /isaac-sim/runheadless.sh
```

Then, in a separate browser window, open `https://{your instance's address}/viewer`.

For example, if this is run on a local computer: `127.0.0.1/viewer`
If this is run on an AWS instance, the address may be more like: 
`http://ec2-00-00-000-000.compute-1.amazonaws.com/viewer`
