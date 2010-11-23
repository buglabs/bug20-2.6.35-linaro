#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/net.h>
#include <linux/inet.h>
#include <linux/skbuff.h>
#include <net/sock.h>

#include <net/inet_common.h>
#include <linux/bmi/bmi_zb.h>
#include "bmi_zigbee.h"

HLIST_HEAD(zaccel_list);
DEFINE_RWLOCK(zaccel_list_lock);

struct zaccel_sock {
	struct sock   sk;
	struct        packet_type zpacket_type;
	spinlock_t    bind_lock;
	struct sockaddr_zb sockaddr;
	struct net_device *dev;

};

static inline struct zaccel_sock *z_sk(struct sock *sk)
{
	return (struct zaccel_sock *)sk;
}

static void zaccel_sock_destruct(struct sock *sk)
{
	if(!sock_flag(sk,SOCK_DEAD)) {
		printk("Attempt to release alive packet socket: %p\n",sk);
		return;
	}
	
	sk_refcnt_debug_dec(sk);
}

static int z_getname(struct socket *sock, struct sockaddr *uaddr, int *uaddr_len, int peer)
{
	struct sock *sk;
	struct zaccel_sock *zsock;
	struct sockaddr_zb *zaddr;

	printk("z_packet_getname\n");
	zaddr = (struct sockaddr_zb *)uaddr;

	sk = sock->sk;
	zsock = z_sk(sk);

	if(zsock->dev == 0)
	{
		
		printk(KERN_WARNING "sock not bound \n");
		/* the socket is not bound */
		return -ENODATA;
	}

	zaddr->z_family = AF_ZACCEL;
	zaddr->z_ifindex = zsock->sockaddr.z_ifindex;
	zaddr->z_protocol = zsock->sockaddr.z_protocol;
	memcpy(&zaddr->z_name, &zsock->sockaddr.z_name, 15);
	
	*uaddr_len = sizeof(struct sockaddr_zb);
	return 0;
	
}

static struct proto zaccel_proto = {
	.name     = "Z_PACKET",
	.owner    =  THIS_MODULE,
	.obj_size = sizeof(struct zaccel_sock),
};

static int z_control_rcv(struct sk_buff *skb, struct net_device *dev, struct packet_type *pt, struct net_device *orig_dev)
{
	struct sock *sk;

	sk = pt->af_packet_priv;

	/* put packet in receive queue */
	if(sock_queue_rcv_skb(sk,skb) == 0)
	{
		return 0;
	}

	printk(KERN_WARNING "z_control_rcv drop\n");
	kfree_skb(skb);
	return 0;

}

static int z_packet_rcv(struct sk_buff *skb, struct net_device *dev, struct packet_type *pt, struct net_device *orig_dev)
{
	struct sock *sk;

	sk = pt->af_packet_priv;

	/*
 	 * put the packet in the receive queue.
 	 */

	if(sock_queue_rcv_skb(sk,skb) == 0)
	{
		return 0;
	}

	printk(KERN_WARNING "z_packet_rcv drop\n");
	kfree_skb(skb);
	return 0;
}

/*
 *	Pull a packet from our recieve queue and hand it to the user.
 */

static int z_recvmsg(struct kiocb *iocb, struct socket *sock,
                     struct msghdr *msg, size_t len, int flags)
{
	struct sock *sk;
	struct sk_buff *skb;
	int skb_len;
	struct net_device *dev;
	struct zaccel_sock *zsock;
	int ifindex;
	int err;

	err = -EINVAL;
	if(flags & ~(MSG_PEEK|MSG_DONTWAIT|MSG_TRUNC|MSG_CMSG_COMPAT))
	{
		goto out;
	}

	sk = sock->sk;

	zsock = z_sk(sk);

	ifindex = zsock->sockaddr.z_ifindex;

	dev = dev_get_by_index(&init_net, ifindex);
	if(dev == NULL)
	{
		printk(KERN_WARNING "bmi_zprotocol: dev not found\n");
		return (-ENXIO);
	}

	if(!(dev->flags & IFF_UP))
	{
		printk("interface not up %d\n",(-ENETDOWN));
		dev_put(dev);
		return -ENETDOWN;
	}

	dev_put(dev);

	/*
	 * Get a datagram skbuff
	 */
	skb = skb_recv_datagram(sk,flags,flags&MSG_DONTWAIT,&err);

	if(skb == NULL)
	{
		goto out;
	}

	/*
	 * If the input buffer is smaller than the message, truncate
	 * it.  The user loses any data beyond it.
	 */
	skb_len = skb->len;
	if(skb_len > len)
	{
		skb_len = len;
		msg->msg_flags |= MSG_TRUNC;
	}
	
	err = skb_copy_datagram_iovec(skb, 0, msg->msg_iov, skb_len);
	if(err)
		goto out_free;

	sock_recv_timestamp(msg, sk, skb);

	/* err returns data  length if the copy is successful */
	err = skb_len;

out_free:
	skb_free_datagram(sk,skb);

out:
	return err;
}

static int z_sendmsg(struct kiocb *iocb, struct socket *sock,
                     struct msghdr *msg, size_t len)
{
	struct sock *sk = sock->sk;
	struct sk_buff *skb;
	struct net_device *dev;
	struct zaccel_sock *zsock;
	int ifindex;
	int err;

	zsock = z_sk(sk);

	if(zsock->zpacket_type.type == Z_CONTROL_SOCK)
	{
		return -EPROTOTYPE;
	}

	ifindex = zsock->sockaddr.z_ifindex;

	dev = dev_get_by_index(&init_net, ifindex);
	if(dev == NULL)
	{
		printk(KERN_WARNING "bmi_zprotocol: dev not found\n");
		dev_put(dev);
		return (-ENXIO);
	}

	if(!(dev->flags & IFF_UP))
	{
		printk(KERN_WARNING "bmi_zprotocol: interface not up %d\n",(-ENETDOWN));
		dev_put(dev);
		return -ENETDOWN;
	}

	if(len > 92)
	{
		/* message is too long */
		dev_put(dev);
		return -EINVAL;
	}

	skb = sock_alloc_send_skb(sk, len, msg->msg_flags & MSG_DONTWAIT, &err);
	if(skb == NULL)
	{
		dev_put(dev);
		printk(KERN_WARNING "bmi_zprotocol: sock_allock_send_skb failed %d\n",err);
		return -ENOMEM;
	}

	err = memcpy_fromiovec(skb_put(skb,len), msg->msg_iov, len);
	if(err)
	{
		kfree_skb(skb);
		dev_put(dev);
		return -EFAULT;
	}
	
	skb->dev = dev;
	
	/*
	 * dev_queue_xmit sends the packet directly to the driver.
	 */

	err = dev_queue_xmit(skb);
	if (err > 0)
	{
		kfree_skb(skb);
		printk(KERN_WARNING "bmi_zprotocol: dev_queue_xmit failed %d\n",err);
		dev_put(dev);
		return(-ENETDOWN);
	}

	dev_put(dev);
	return(len);
}


static int z_bind(struct socket *sock, struct sockaddr *uaddr, int addr_len)
{
	struct sock *sk = sock->sk;
	char name[15];
	struct net_device *dev;
	struct sockaddr_zb *z_addr;
	struct zaccel_sock *zsock;
	struct net_zb *priv;
	unsigned char type;
	int rc;

	z_addr = (struct sockaddr_zb *)uaddr;

	/* Get the name of the device */
	strlcpy(name,z_addr->z_name,sizeof(name));

	rc = (int)strlen(name);
	if(rc > 3)
	{
		return -EINVAL;
	}

	if(memcmp(name,"zb",2))
	{
		printk(KERN_WARNING "bmi_zprotocol: invalid name %s\n",name);
		return -EINVAL;
	}

	if((name[2] < 0x31) && (name[2] > 0x34))
	{
		printk(KERN_WARNING "bmi_zprotocol: invalid slot %c\n",name[2]);
		return -EINVAL;
	}


	zsock = z_sk(sk);
	type =  zsock->zpacket_type.type;
	
	/* search for the network interface by name */
	dev = dev_get_by_name(&init_net, name);

	/* 
	 * check if the device has been bound to this socket type.
	 * If it has, return with error.
	 */

	priv = netdev_priv(dev);
	if(priv->socket[type] != Z_NO_SOCK)
	{
		dev_put(dev);
		return -EISCONN;
	}
	
	if(dev) 
	{
		lock_sock(sk);
		spin_lock(&zsock->bind_lock);

		if(zsock->dev && (zsock->dev != dev))
		{
			/* This socket was bound.   Unbind it first.  */
			printk(KERN_INFO "unbound to previous device\n");
			__sock_put(sk);

			priv = netdev_priv(zsock->dev);
			priv->socket[type] = Z_NO_SOCK;
		
			zsock->dev = NULL;
			spin_unlock(&zsock->bind_lock);
			dev_remove_pack(&zsock->zpacket_type);
			spin_lock(&zsock->bind_lock);
		}

		zsock->zpacket_type.dev = dev;
		zsock->zpacket_type.af_packet_priv = sk;

		/* socket-dev information used by sendmsg */
		zsock->sockaddr.z_ifindex = dev->ifindex;
		strlcpy(zsock->sockaddr.z_name,name,15);

		zsock->dev = dev;

		/* Add a packet handler to the networking stack. */
		dev_add_pack(&zsock->zpacket_type);
		/* increment sk_refcnt */
		sock_hold(sk);

		/* Tell the device that it is bound to a socket */
		priv = netdev_priv(dev);
		priv->socket[type] = 1;
	
		spin_unlock(&zsock->bind_lock);
		release_sock(sk);
		dev_put(dev);
		return 0;
	}
	else
	{
		printk(KERN_WARNING "zb: dev %s not found\n",name);
		return -ENODEV;
	}
}

static int z_ioctl(struct socket *sock, unsigned int cmd, unsigned long arg)
{
	int rc;

	switch(cmd) 
	{
		case SIOCGIFFLAGS:
		case SIOCGIFNAME:
		case SIOCGIFMTU:
		case SIOCGIFINDEX:
		case SIOCETHTOOL:
		case SIOCSIFNAME:
		case SIOCSIFFLAGS:
		case SIOCSIFMTU:
			/*
 			 * return -ENOIOCTLCMD to sock_ioctl
 			 * sock_ioctl will call dev_ioctl to take care of these cmds.
 			 */
			rc = -ENOIOCTLCMD;
			break;
		default:
			rc = 0;
			break;
	}

	return rc;
}

static int z_setsockopt(struct socket *sock, int level, int optname, char __user *optval, int optlen)
{
	struct sock *sk = sock->sk;
	struct zaccel_sock *zsock;
	unsigned char buf[135];  /* 128 data + 6 fixed header + 1 just in case */
	
	zsock = z_sk(sk);

	if(!zsock->dev)	
	{
		printk(KERN_WARNING "bmi_zprotocol: device not attached\n");
		/*
 		 * socket has no device attached
 		 */
		return -ENOTCONN;
	}

	if(level != SOL_ZACCEL)
	{
		return -ENOPROTOOPT;
	}

	if(optlen > 135)
	{
		return -EINVAL;
	}
	
	if(copy_from_user(buf,optval,optlen))
	{
		return -EFAULT;
	}

	if(zdev_setopt(zsock->dev,optname,optlen,buf))
	{
		return -EINVAL;
	}
	
	return 0;
}

static int z_getsockopt(struct socket *sock, int level, int optname,
                        char __user *optval, int __user *optlen)

{
	struct sock *sk = sock->sk;
	struct zaccel_sock *zsock;
	unsigned char buf[135];  /* 128 data + 6 fixed header + 1 just in case */
	int len;
	int rc;

	zsock = z_sk(sk);

	if(!zsock->dev)
	{
		/* socket has no device attached */
		return -ENOTCONN;
	}
	
	if(level != SOL_ZACCEL)
		return -ENOPROTOOPT;

	rc = zdev_getopt(zsock->dev,optname,&len,buf);

	if(!rc)
	{
		if(put_user(len, optlen))
			return -EFAULT;
		/* Include 3 header bytes to the message */
		if(copy_to_user(optval, buf, (len+3)))
			return -EFAULT;
	}

	return rc;
	
}

static int z_release(struct socket *sock)
{
	struct sock *sk;
	struct zaccel_sock *zsock;
	struct net_zb *priv;

	sk = sock->sk;

	if(!sk)
		return 0;

	zsock = z_sk(sk);

	write_lock_bh(&zaccel_list_lock);
	sk_del_node_init(sk);
	write_unlock_bh(&zaccel_list_lock);

	if(zsock->dev)
	{
		priv = netdev_priv(zsock->dev);
		priv->socket[zsock->zpacket_type.type] = Z_NO_SOCK;

		/* remove protocol handler */
		dev_remove_pack(&zsock->zpacket_type);
		__sock_put(sk);
	}

	/* detach socket from process context */
	sock_orphan(sk);
	sock->sk = NULL;

	/* Purge queues */
	skb_queue_purge(&sk->sk_receive_queue);
	
	sk_refcnt_debug_release(sk);

	sock_put(sk);
	return 0;
}

static const struct proto_ops z_protocol_ops = {
	.family =      PF_ZACCEL,
	.owner  =      THIS_MODULE,
	.release =     z_release,
	.bind =        z_bind,
	.connect =     sock_no_connect,
	.socketpair =  sock_no_socketpair,
	.accept =      sock_no_accept,
	.getname =     z_getname,
	.poll =        sock_no_poll,
	.ioctl =       z_ioctl,
	.listen =      sock_no_listen,
	.shutdown =    sock_no_shutdown,
	.setsockopt =  z_setsockopt,
	.getsockopt =  z_getsockopt,
	.sendmsg =     z_sendmsg,
	.recvmsg =     z_recvmsg,
	.mmap =        sock_no_mmap,
	.sendpage =    sock_no_sendpage,
};

int z_protocol_create(struct net *net, struct socket *sock, int protocol)
{
	struct sock *sk;
	struct zaccel_sock *zsock;

	if (net != &init_net)
	{
		return -EAFNOSUPPORT;
	}

	sock->state = SS_UNCONNECTED;
	if(sock->type != SOCK_RAW) 
	{
		return -ESOCKTNOSUPPORT;
	}

	sock->ops = &z_protocol_ops;

	sk = sk_alloc(net, AF_ZACCEL, GFP_KERNEL, &zaccel_proto);
	if(sk == NULL)
	{
		return -ENOMEM;
	}

	sock_init_data(sock, sk);
	sk->sk_protocol = protocol;
	sk->sk_family = PF_ZACCEL;
	sk->sk_destruct = zaccel_sock_destruct;
	sk_refcnt_debug_inc(sk);

	zsock = z_sk(sk);

	spin_lock_init(&zsock->bind_lock);

	if(protocol == Z_CONTROL_SOCK)
	{
		zsock->zpacket_type.func = z_control_rcv;
		zsock->zpacket_type.type = Z_CONTROL_SOCK;
		sk->sk_protocol = Z_CONTROL_SOCK;
	}
	else
	{
		zsock->zpacket_type.func = z_packet_rcv;
		zsock->zpacket_type.type = Z_PACKET_SOCK;
		sk->sk_protocol = Z_PACKET_SOCK;
	}
	zsock->zpacket_type.af_packet_priv = sk;
	zsock->zpacket_type.dev = NULL;

	zsock->dev = NULL;

	/* Add a socket to the bound sockets list */
	write_lock_bh(&zaccel_list_lock);
	sk_add_node(sk,&zaccel_list);
	write_unlock_bh(&zaccel_list_lock);

	return 0;
		
}

static struct net_proto_family zaccel_family_ops = {
	.family  = PF_ZACCEL,
	.create  = z_protocol_create,
	.owner   = THIS_MODULE,
};

void z_sock_exit(void)
{
	sock_unregister(PF_ZACCEL);
}

int z_sock_init(void)
{
	int res;

	res = sock_register(&zaccel_family_ops);
	if(res)	{
		printk(KERN_WARNING "Failed to register PF_ZACCEL\n");
	}

	return res;
}
