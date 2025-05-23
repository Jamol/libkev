/* Copyright (c) 2016, Fengping Bao <jamol@live.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include "kevdefs.h"

#ifdef KUMA_OS_WIN
# include <Ws2tcpip.h>
# include <windows.h>
#elif defined(KUMA_OS_LINUX) || defined(KUMA_OS_OHOS)
# include <string.h>
# include <pthread.h>
# include <unistd.h>
# include <fcntl.h>
# include <sys/types.h>
# include <sys/stat.h>
# include <sys/time.h>
# include <sys/socket.h>
# include <netdb.h>
# include <signal.h>
# include <arpa/inet.h>
# include <netinet/tcp.h>
# include <netinet/in.h>

#elif defined(KUMA_OS_MAC)
# include <string.h>
# include <pthread.h>
# include <unistd.h>

# include <sys/types.h>
# include <sys/socket.h>
# include <sys/ioctl.h>
# include <sys/fcntl.h>
# include <sys/time.h>
# include <sys/uio.h>
# include <netinet/tcp.h>
# include <netinet/in.h>
# include <arpa/inet.h>
# include <netdb.h>
# include <ifaddrs.h>

#else
# error "UNSUPPORTED OS"
#endif

#if defined(KUMA_OS_LINUX) || defined(KUMA_OS_OHOS)
#include "EventNotifier.h"
#elif !defined(KUMA_OS_WIN)
#include "PipeNotifier.h"
#else
#include "SocketNotifier.h"
#endif

KEV_NS_USING

NotifierPtr Notifier::createNotifier() {
#if defined(KUMA_OS_LINUX) || defined(KUMA_OS_OHOS)
    return NotifierPtr(new EventNotifier());
#elif !defined(KUMA_OS_WIN)
    return NotifierPtr(new PipeNotifier());
#else
    return NotifierPtr(new SocketNotifier());
#endif
}
